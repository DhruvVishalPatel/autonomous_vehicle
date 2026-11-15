"""Unscented Kalman Filter for the robot's pose, with online motor calibration.

State  x = [px, py, theta, kv, kw]
  px, py, theta   position (m) and heading (rad) on the map
  kv              m/s the robot actually drives per unit of forward command
  kw              rad/s it actually turns per unit of turn command
The robot has no wheel encoders and its true speeds are unknown, so kv and kw
are estimated along with the pose: the filter calibrates the motors as it drives.

Predict (20 Hz, control loop), from the wheel commands (left, right in -1..1):
  u_fwd = (left + right) / 2,  u_turn = (right - left) / 2
  v = kv * u_fwd,  omega = kw * u_turn,  then constant turn-rate/velocity motion.
  Process noise covers wheel slip and the gains drifting (battery, floor).
Update (~3.3 Hz, each lidar scan): scan-matching pose z = [px, py, theta].
  Noise grows when match quality is low; matches that disagree wildly with the
  prediction (Mahalanobis gate) are rejected as outliers (never more than 3 in a row).

The unscented transform pushes 2n+1 sigma points through the non-linear motion
model instead of linearising it (that would be an EKF).
"""
import math

import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PoseUKF:
    N = 5

    def __init__(self, cfg):
        u = cfg["ukf"]
        self.slip = u["slip_noise"]                  # m/s     unmodelled motion (slip, bumps)
        self.yaw_slip = u["yaw_slip_noise"]          # rad/s   unmodelled rotation
        self.gain_drift = u["gain_noise"]            # per s   how fast kv/kw may drift
        self.kv0, self.kw0 = u["init_speed_gain"], u["init_turn_gain"]
        self.sig_pos = u["meas_pos_noise"]           # m       scan-match position noise
        self.sig_th = math.radians(u["meas_angle_noise_deg"])
        self.gate = u["gate"]                        # Mahalanobis^2 threshold (3 dof)
        n = self.N
        self.lam = 0.0                               # alpha=1, beta=2, kappa=0
        self.wm = np.full(2 * n + 1, 1.0 / (2 * n))
        self.wc = self.wm.copy()
        self.wm[0], self.wc[0] = 0.0, 2.0
        self.u = (0.0, 0.0)
        self.kv = self.kw = None
        self.reset()

    def reset(self, pose=(0.0, 0.0, 0.0), keep_gains=True):
        kv = self.x[3] if (keep_gains and self.kv is not None) else self.kv0
        kw = self.x[4] if (keep_gains and self.kw is not None) else self.kw0
        self.x = np.array([pose[0], pose[1], pose[2], kv, kw], float)
        self.P = np.diag([0.01 ** 2, 0.01 ** 2, 0.01 ** 2, (0.5 * kv) ** 2, (0.5 * kw) ** 2])
        self.kv, self.kw = kv, kw
        self.rejected, self.last_nis = 0, 0.0

    # ------------------------------------------------------------ helpers
    def _sigma(self):
        n = self.N
        S = np.linalg.cholesky((n + self.lam) * (self.P + 1e-12 * np.eye(n)))
        X = np.tile(self.x, (2 * n + 1, 1))
        X[1:n + 1] += S.T
        X[n + 1:] -= S.T
        return X

    @staticmethod
    def _mean(X, w, angle_idx):
        m = w @ X
        for i in angle_idx:
            m[i] = math.atan2(w @ np.sin(X[:, i]), w @ np.cos(X[:, i]))
        return m

    @staticmethod
    def _f(x, dt, uf, ut):
        px, py, th, kv, kw = x
        v, w = kv * uf, kw * ut
        if abs(w) > 1e-4:
            px += v / w * (math.sin(th + w * dt) - math.sin(th))
            py += v / w * (math.cos(th) - math.cos(th + w * dt))
        else:
            px += v * math.cos(th) * dt
            py += v * math.sin(th) * dt
        return np.array([px, py, wrap(th + w * dt), kv, kw])

    # ------------------------------------------------------------ predict
    def predict(self, dt, left, right):
        """Advance by dt seconds with the wheel commands that were applied."""
        if dt <= 0:
            return
        uf, ut = (left + right) / 2.0, (right - left) / 2.0
        self.u = (uf, ut)
        X = np.array([self._f(s, dt, uf, ut) for s in self._sigma()])
        self.x = self._mean(X, self.wm, [2])
        d = X - self.x
        d[:, 2] = (d[:, 2] + math.pi) % (2 * math.pi) - math.pi
        moving = abs(uf) + abs(ut) > 1e-3
        s_p = self.slip * dt if moving else 0.002 * dt       # parked robots don't slide
        s_t = self.yaw_slip * dt if moving else 0.002 * dt
        Q = np.diag([s_p ** 2, s_p ** 2, s_t ** 2,
                     (self.gain_drift * self.x[3]) ** 2 * dt, (self.gain_drift * self.x[4]) ** 2 * dt])
        self.P = (self.wc[:, None] * d).T @ d + Q
        self.P = 0.5 * (self.P + self.P.T)
        self.x[3] = max(0.02, self.x[3])                      # gains stay positive
        self.x[4] = max(0.05, self.x[4])

    # ------------------------------------------------------------ update
    def update_pose(self, z, quality=1.0):
        """Fuse a scan-matching pose. Returns True if accepted, False if gated out."""
        q = max(0.1, min(1.0, quality))
        R = np.diag([(self.sig_pos / q) ** 2] * 2 + [(self.sig_th / q) ** 2])
        X = self._sigma()
        Z = X[:, :3]
        zm = self._mean(Z, self.wm, [2])
        dz = Z - zm
        dz[:, 2] = (dz[:, 2] + math.pi) % (2 * math.pi) - math.pi
        dx = X - self.x
        dx[:, 2] = (dx[:, 2] + math.pi) % (2 * math.pi) - math.pi
        S = (self.wc[:, None] * dz).T @ dz + R
        C = (self.wc[:, None] * dx).T @ dz
        y = np.asarray(z, float) - zm
        y[2] = wrap(y[2])
        Si = np.linalg.inv(S)
        self.last_nis = float(y @ Si @ y)
        if self.last_nis > self.gate and self.rejected < 3:
            self.rejected += 1
            return False
        self.rejected = 0
        K = C @ Si
        self.x = self.x + K @ y
        self.x[2] = wrap(self.x[2])
        self.x[3] = max(0.02, self.x[3])
        self.x[4] = max(0.05, self.x[4])
        self.P = self.P - K @ S @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        return True

    # ------------------------------------------------------------ outputs
    @property
    def pose(self):
        return self.x[:3].copy()

    def sigma(self):
        """(position std-dev m, heading std-dev rad, current speed m/s, current turn rate rad/s)."""
        return (math.sqrt(max(self.P[0, 0], self.P[1, 1])), math.sqrt(self.P[2, 2]),
                float(self.x[3] * self.u[0]), float(self.x[4] * self.u[1]))

    def gains(self):
        return float(self.x[3]), float(self.x[4])
