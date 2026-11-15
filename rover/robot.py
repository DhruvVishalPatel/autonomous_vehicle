"""Ties lidar + motors + safety together with a 20 Hz control loop."""
import math
import threading
import time
from pathlib import Path

import numpy as np

from rover.camera import Camera
from rover.navigator import Navigator
from rover.ukf import PoseUKF
from rover.safety import Safety
from rover.slam import SLAM, compose, relative

MAP_DIR = Path(__file__).resolve().parent.parent / "maps"


class Robot:
    def __init__(self, cfg, fake=False):
        self.cfg = cfg
        if fake:
            from rover.fake import FakeLidar, FakeMotors
            self.lidar, self.motors = FakeLidar(cfg), FakeMotors(cfg)
        else:
            from rover.lidar import Lidar
            from rover.motors import Motors
            self.lidar, self.motors = Lidar(cfg), Motors(cfg)
        self.safety = Safety(cfg)
        self.slam = SLAM(cfg)
        self.slam_ms = 0.0
        self.messages = []
        self._scan_evt = threading.Event()
        self.lidar.on_scan = lambda s: self._scan_evt.set()
        self.nav = Navigator(cfg, self.slam, log=self.log)
        self.ukf = PoseUKF(cfg)
        self._ukf_lock = threading.Lock()
        self.camera = Camera(cfg) if not fake else Camera({})
        if cfg.get("camera", {}).get("enabled") and not self.camera.enabled and not fake:
            self.log(f"camera off: {self.camera.error}")
        d = cfg["drive"]
        self.vmax, self.wmax = d["max_speed"], d["turn_speed"]
        self.manual_timeout, self.lidar_timeout = d["manual_timeout"], d["lidar_timeout"]
        self.estop = False
        self._joy = (0.0, 0.0, 0.0)      # v, w, time
        self.cmd = (0.0, 0.0)            # wheels actually sent (left, right)
        self.status = "starting"
        self.loop_hz = 0.0
        self._run = True
        threading.Thread(target=self._loop, name="control", daemon=True).start()
        threading.Thread(target=self._slam_loop, name="slam", daemon=True).start()
        from rover import slam as _slam
        if _slam.cv2 is None:
            self.log("WARNING: OpenCV missing -> mapping is very slow. Run ./setup_venv.sh")

    # ---------------------------------------------------------- inputs
    def joystick(self, x, y):
        """x: -1 (left) .. 1 (right), y: -1 (back) .. 1 (forward)."""
        x, y = max(-1.0, min(1.0, float(x))), max(-1.0, min(1.0, float(y)))
        self._joy = (y * self.vmax, -x * self.wmax, time.monotonic())

    def set_estop(self, on):
        self.estop = bool(on)
        if on:
            self.nav.cancel("E-STOP")
            self.motors.stop()

    def go_to(self, x, y):
        self.nav.go_to(x, y)

    def explore(self):
        self.nav.explore()

    def pose(self):
        """Fused pose from the UKF (updated 20x per second)."""
        with self._ukf_lock:
            return self.ukf.pose

    def log(self, msg):
        t = time.strftime("%H:%M:%S")
        print(f"[{t}] {msg}", flush=True)
        self.messages = (self.messages + [f"{t} {msg}"])[-8:]

    # ---------------------------------------------------------- mapping
    def _slam_loop(self):
        """Per lidar scan: the UKF's predicted pose seeds the scan matcher, the
        matched pose is fused back into the UKF, and the map uses the fused pose."""
        last_stamp = None
        while self._run:
            if not self._scan_evt.wait(1.0):
                continue
            self._scan_evt.clear()
            scan = self.lidar.latest
            if scan is None or scan.stamp == last_stamp:
                continue
            last_stamp = scan.stamp
            with self._ukf_lock:
                prior = self.ukf.pose
                turn_rate = self.ukf.sigma()[3]
            pred = relative(prior, self.slam.pose)       # motion since the last scan, per UKF
            t0 = time.perf_counter()
            try:
                matched = self.slam.process(scan, pred, turn_rate=turn_rate)
            except Exception as e:
                self.log(f"mapping error: {e}")
                continue
            with self._ukf_lock:
                if self.slam.scans > 1 and not self.slam.lost:
                    self.ukf.update_pose(matched, self.slam.quality)
                fused = self.ukf.pose
            self.slam.pose = fused                       # next scan starts from the fused pose
            self.slam_ms = 0.8 * self.slam_ms + 0.2 * (time.perf_counter() - t0) * 1000

    def save_map(self, name):
        name = "".join(ch for ch in str(name) if ch.isalnum() or ch in "-_") or "map"
        MAP_DIR.mkdir(exist_ok=True)
        self.slam.save(MAP_DIR / f"{name}.npz")
        self.log(f"map saved: {name}")
        return name

    def load_map(self, name):
        try:
            self.nav.cancel("map loaded")
            self.slam.load(MAP_DIR / f"{name}.npz")
            with self._ukf_lock:
                self.ukf.reset()
            self.log(f"map loaded: {name} (robot must be back at that map's start spot, same direction)")
        except Exception as e:
            self.log(f"could not load map '{name}': {e}")

    def list_maps(self):
        MAP_DIR.mkdir(exist_ok=True)
        return sorted(p.stem for p in MAP_DIR.glob("*.npz"))

    def clear_map(self):
        self.nav.cancel("map cleared")
        self.slam.reset()
        with self._ukf_lock:
            self.ukf.reset()
        self.log("map cleared — current position is the new origin")

    def map_bytes(self):
        """Cropped map: 4 x int32 (row0, col0, h, w) + uint8 cells (0..254 occupancy, 255 unknown)."""
        L = self.slam.grid.L
        seen = np.abs(L) > 0.2
        rows, cols = np.where(seen.any(axis=1))[0], np.where(seen.any(axis=0))[0]
        if rows.size == 0:
            return np.zeros(4, "<i4").tobytes()
        r0, r1 = max(rows[0] - 2, 0), min(rows[-1] + 3, L.shape[0])
        c0, c1 = max(cols[0] - 2, 0), min(cols[-1] + 3, L.shape[1])
        sub = L[r0:r1, c0:c1]
        cells = (254.0 / (1.0 + np.exp(-sub))).astype(np.uint8)
        cells[np.abs(sub) <= 0.2] = 255
        return np.array([r0, c0, r1 - r0, c1 - c0], "<i4").tobytes() + cells.tobytes()

    # ---------------------------------------------------------- loop
    def _loop(self):
        dt = 0.05
        last = time.monotonic()
        while self._run:
            t0 = time.monotonic()
            with self._ukf_lock:                          # UKF predict at 20 Hz with the wheels as driven
                self.ukf.predict(t0 - last, *self.cmd)
            last = t0
            v, w, tj = self._joy
            scan = self.lidar.latest
            if self.estop:
                v = w = 0.0
                self.status = "E-STOP"
            elif scan is None or t0 - scan.stamp > self.lidar_timeout:
                v = w = 0.0
                self.status = "no lidar data"
            elif t0 - tj < self.manual_timeout and (v or w):
                if self.nav.active:
                    self.nav.cancel("cancelled: joystick used")
                v, w = self.safety.filter(v, w, scan)
                self.status = self.safety.reason or "driving"
            elif self.nav.active:
                v, w = self.nav.command(self.pose(), self.safety, scan)
                v, w = self.safety.filter(v, w, scan)
                self.status = self.safety.reason or self.nav.status
            else:
                v = w = 0.0
                self.safety.filter(0, 0, scan)        # keep clearances updated
                self.status = "idle"
            left, right = v - w, v + w
            m = max(abs(left), abs(right), 1.0)
            left, right = left / m, right / m
            self.cmd = (left, right)
            if left == 0 and right == 0:
                self.motors.stop()
            else:
                self.motors.drive(left, right)
            el = time.monotonic() - t0
            self.loop_hz = 0.9 * self.loop_hz + 0.1 / max(dt, el)
            time.sleep(max(0.0, dt - el))

    def state(self):
        s = self.lidar.latest
        pts = []
        if s is not None:
            xy = s.points()
            step = max(1, len(xy) // 400)
            pts = [[round(a, 3), round(b, 3)] for a, b in xy[::step]]
        f, r = self.safety.front, self.safety.rear
        g, p = self.slam.grid, self.pose()
        with self._ukf_lock:
            sp, sth, vel, om = self.ukf.sigma()
        return {
            "pose": [round(float(v), 3) for v in p],
            "ukf": {"pos_cm": round(sp * 100, 1), "heading_deg": round(math.degrees(sth), 1),
                    "speed": round(vel, 2), "turn": round(om, 2),
                    "gains": [round(g, 2) for g in self.ukf.gains()]},
            "mode": self.nav.mode,
            "camera": self.camera.enabled,
            "slam": {"quality": round(float(self.slam.quality), 2), "lost": bool(self.slam.lost),
                     "ms": round(self.slam_ms), "scans": int(self.slam.scans),
                     "mapping": bool(abs(self.slam.turn_rate) <= self.slam.max_turn)},
            "map": {"res": float(g.res), "origin": float(g.origin), "version": int(g.version)},
            "messages": self.messages,
            "goal": self.nav.goal,
            "path": [[round(a, 2), round(b, 2)] for a, b in (self.nav.path or [])[::2]],
            "status": self.status,
            "estop": self.estop,
            "points": pts,
            "front": None if math.isinf(f) else round(f, 2),
            "rear": None if math.isinf(r) else round(r, 2),
            "cmd": [round(self.cmd[0], 2), round(self.cmd[1], 2)],
            "lidar_hz": round(self.lidar.rate_hz, 1),
            "loop_hz": round(self.loop_hz, 1),
            "robot": {"length": self.cfg["robot"]["length"], "width": self.cfg["robot"]["width"],
                      "stop": self.cfg["safety"]["stop_distance"], "slow": self.cfg["safety"]["slow_distance"]},
        }

    def close(self):
        self._run = False
        time.sleep(0.1)
        self.motors.close()
        self.lidar.close()
