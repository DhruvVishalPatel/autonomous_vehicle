"""Lightweight 2D lidar SLAM (no encoders needed).

* Map: log-odds occupancy grid, robot starts at the centre facing +x.
* Localisation: correlative scan-to-map matching. A constant-velocity guess
  (keep moving like the last scan, or stand still if the joystick is released)
  is refined by a coarse-to-fine search for the pose where the scan best lines
  up with walls already in the map.
* Mapping: the scan is ray-traced into the grid once the robot has moved a bit.

All numpy, ~20-60 ms per scan on a Pi 5.
"""
import math
import threading
import time

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

L_OCC = 0.9
L_FREE = -0.4
L_MIN, L_MAX = -4.0, 3.0     # low ceiling: things that leave (people) clear after a few passes
# A cell seen as empty floor this confidently (several clear passes) is not turned into
# a wall by a new hit: whatever is standing there now is treated as moving (a person).
# It is still avoided while driving (live obstacles), it just isn't stored in the map.
FREE_LOCK = -1.5


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(p, d):
    """p ⊕ d : apply a robot-frame delta d to pose p."""
    c, s = math.cos(p[2]), math.sin(p[2])
    return np.array([p[0] + c * d[0] - s * d[1], p[1] + s * d[0] + c * d[1], wrap(p[2] + d[2])])


def relative(a, b):
    """a ⊖ b : pose a expressed in the frame of pose b."""
    c, s = math.cos(b[2]), math.sin(b[2])
    dx, dy = a[0] - b[0], a[1] - b[1]
    return np.array([c * dx + s * dy, -s * dx + c * dy, wrap(a[2] - b[2])])


class OccupancyGrid:
    def __init__(self, resolution, size_m):
        self.res = float(resolution)
        self.n = int(round(size_m / resolution))
        self.origin = -self.n * self.res / 2.0     # world coord of cell (0,0) corner
        self.L = np.zeros((self.n, self.n), dtype=np.float32)
        self.version = 0

    def world_to_cell(self, xy):
        ij = np.floor((xy - self.origin) / self.res).astype(np.int32)
        return ij[..., 1], ij[..., 0]      # row (y), col (x)

    def cell_to_world(self, r, c):
        return (np.asarray(c) + 0.5) * self.res + self.origin, (np.asarray(r) + 0.5) * self.res + self.origin

    def inside(self, r, c):
        return (r >= 0) & (r < self.n) & (c >= 0) & (c < self.n)

    def update(self, pose, pts, hit):
        """pts: Nx2 robot-frame endpoints, hit: bool mask (False = max-range ray)."""
        c, s = math.cos(pose[2]), math.sin(pose[2])
        R = np.array([[c, -s], [s, c]])
        wpts = pts @ R.T + pose[:2]
        # cells hit this scan, and which of them are known empty floor (-> moving thing, skip)
        hr, hc = self.world_to_cell(wpts[hit])
        hok = self.inside(hr, hc)
        occ_flat = np.unique(hr[hok] * self.n + hc[hok])
        occ_flat = occ_flat[self.L.ravel()[occ_flat] > FREE_LOCK]
        # --- free space: sample along each ray up to one cell before the hit
        rng = np.hypot(pts[:, 0], pts[:, 1])
        step = self.res * 0.8
        k = np.maximum((rng - self.res) / step, 0).astype(np.int32)
        total = int(k.sum())
        if total > 0:
            beam = np.repeat(np.arange(len(k)), k)
            start = np.cumsum(k) - k
            t = (np.arange(total) - np.repeat(start, k)) * step / np.repeat(rng, k)
            fx = pose[0] + (wpts[beam, 0] - pose[0]) * t
            fy = pose[1] + (wpts[beam, 1] - pose[1]) * t
            r, cc = self.world_to_cell(np.column_stack((fx, fy)))
            ok = self.inside(r, cc)
            flat = np.unique(r[ok] * self.n + cc[ok])
            self.L.ravel()[flat] += L_FREE
        # --- occupied endpoints (except on known empty floor)
        self.L.ravel()[occ_flat] += L_OCC
        np.clip(self.L, L_MIN, L_MAX, out=self.L)
        self.version += 1

    def prob(self):
        return 1.0 / (1.0 + np.exp(-self.L))


class LikelihoodField:
    """Smooth 'closeness to a wall' score in [0,1] used for scan matching."""

    def __init__(self, grid, sigma=0.06):
        self.grid = grid
        self.sigma = sigma
        self.field = np.zeros_like(grid.L)
        self.version = -1

    def refresh(self):
        g = self.grid
        if g.version == self.version:
            return
        occ = g.L > 0.6
        if cv2 is not None:
            dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 3) * g.res
        else:
            dist = _numpy_distance(occ, g.res, max_cells=4)
        self.field = np.exp(-(dist ** 2) / (2 * self.sigma ** 2)).astype(np.float32)
        self.version = g.version


def _numpy_distance(occ, res, max_cells=4):
    d = np.full(occ.shape, max_cells * res * 2, dtype=np.float32)
    for dy in range(-max_cells, max_cells + 1):
        for dx in range(-max_cells, max_cells + 1):
            sh = np.roll(np.roll(occ, dy, 0), dx, 1)
            d = np.where(sh, np.minimum(d, math.hypot(dx, dy) * res), d)
    return d


class SLAM:
    def __init__(self, cfg):
        s = cfg["slam"]
        self.grid = OccupancyGrid(s["resolution"], s["size_m"])
        self.field = LikelihoodField(self.grid)
        self.search_xy = s["search_xy"]
        self.search_th = math.radians(s["search_theta_deg"])
        self.min_points = s["min_points"]
        self.upd_d = s["update_dist"]
        self.upd_a = math.radians(s["update_angle_deg"])
        self.max_turn = s["max_turn_for_mapping"]     # rad/s; above this, localise only
        self.prior_weight = s.get("prior_weight", 0.03)
        self.max_range = cfg["lidar"]["max_range"]
        self.turn_rate = 0.0
        self.pose = np.zeros(3)
        self._last_insert = None
        self.quality = 0.0        # mean match score of last scan (0..1)
        self.lost = False
        self.scans = 0
        self.lock = threading.Lock()

    # -------------------------------------------------------------- public
    def reset(self, pose=(0.0, 0.0, 0.0)):
        with self.lock:
            self.grid.L[:] = 0
            self.grid.version += 1
            self.pose = np.array(pose, dtype=float)
            self._last_insert = None
            self.scans = 0

    def process(self, scan, odom_delta, turn_rate=0.0):
        """Update pose (and map) from one scan. odom_delta = predicted motion since last scan.
        turn_rate (rad/s): scans taken while spinning faster than max_turn are smeared,
        so they are used for position only, not added to the map."""
        self.turn_rate = turn_rate
        T = time.perf_counter
        t0 = T()
        self.timing = {}
        pts = scan.points()
        if len(pts) < self.min_points:
            with self.lock:
                self.pose = compose(self.pose, odom_delta)
            return self.pose
        hit = scan.ranges < self.max_range - 0.05
        # never trust a guess bigger than the robot could physically move between scans
        d = np.asarray(odom_delta, float).copy()
        dist = math.hypot(d[0], d[1])
        lim = 2.0 * self.search_xy          # the UKF prediction is reliable; only cap absurd values
        if dist > lim:
            d[:2] *= lim / dist
        d[2] = max(-math.pi / 2, min(math.pi / 2, d[2]))
        with self.lock:
            pred = compose(self.pose, d)
            if self._last_insert is None:
                est = pred
                self.quality = 1.0
            else:
                t1 = T()
                self.field.refresh()
                self.timing["field"] = (T() - t1) * 1000
                t1 = T()
                est, q = self._match(pts[hit], pred)
                self.timing["match"] = (T() - t1) * 1000
                self.quality = q
                self.lost = q < 0.18
                if self.lost:
                    est = self.pose.copy()   # bad match: hold position, don't extrapolate
            self.pose = est
            if self._should_insert(est) and abs(turn_rate) <= self.max_turn:
                t1 = T()
                self.grid.update(est, pts, hit)
                self.timing["map_update"] = (T() - t1) * 1000
                self._last_insert = est.copy()
            self.scans += 1
            self.timing["total"] = (T() - t0) * 1000
            return self.pose.copy()

    def save(self, path):
        with self.lock:
            np.savez_compressed(path, L=self.grid.L, res=self.grid.res, pose=self.pose)

    def load(self, path, pose=None):
        d = np.load(path)
        L, res = d["L"], float(d["res"])
        if abs(res - self.grid.res) > 1e-9:
            raise ValueError(f"map was saved at {res} m/cell, config uses {self.grid.res}")
        n_old, n = L.shape[0], self.grid.n
        with self.lock:
            # both maps are centred on the start point, so a different size just pads/crops evenly
            L = np.clip(L, L_MIN, L_MAX)       # old maps used a higher ceiling
            self.grid.L[:] = 0
            if n_old <= n:
                o = (n - n_old) // 2
                self.grid.L[o:o + n_old, o:o + n_old] = L
            else:
                o = (n_old - n) // 2
                self.grid.L[:] = L[o:o + n, o:o + n]
            self.grid.version += 1
            # Robot must be placed back at the spot where mapping started
            self.pose = np.array(pose if pose is not None else (0.0, 0.0, 0.0), dtype=float)
            self._last_insert = self.pose.copy()

    # ------------------------------------------------------------ internals
    def _should_insert(self, p):
        if self._last_insert is None:
            return True
        d = relative(p, self._last_insert)
        return math.hypot(d[0], d[1]) > self.upd_d or abs(d[2]) > self.upd_a

    def _score(self, pts, x, y, th_list):
        """Score many candidate poses. Returns array [len(th_list), len(x)*len(y)]."""
        g = self.grid
        F = self.field.field
        gx, gy = np.meshgrid(x, y, indexing="ij")
        txy = np.column_stack((gx.ravel(), gy.ravel()))         # T x 2
        out = np.empty((len(th_list), len(txy)), dtype=np.float32)
        inv = 1.0 / g.res
        for i, th in enumerate(th_list):
            c, s = math.cos(th), math.sin(th)
            rx = pts[:, 0] * c - pts[:, 1] * s
            ry = pts[:, 0] * s + pts[:, 1] * c
            col = ((rx[None, :] + txy[:, :1]) - g.origin) * inv
            row = ((ry[None, :] + txy[:, 1:]) - g.origin) * inv
            col = np.clip(col.astype(np.int32), 0, g.n - 1)
            row = np.clip(row.astype(np.int32), 0, g.n - 1)
            out[i] = F[row, col].mean(axis=1)
        return out, txy

    def _match(self, pts, pred):
        self.field.refresh()
        if len(pts) > 220:
            pts = pts[np.linspace(0, len(pts) - 1, 220).astype(int)]
        best = pred
        # (half-width xy, step xy, half-width th, step th)
        for hw, st, hth, sth in ((self.search_xy, 0.05, self.search_th, math.radians(2.0)),
                                 (0.05, 0.0125, math.radians(2.0), math.radians(0.5))):
            xs = best[0] + np.arange(-hw, hw + 1e-9, st)
            ys = best[1] + np.arange(-hw, hw + 1e-9, st)
            ths = best[2] + np.arange(-hth, hth + 1e-9, sth)
            sc, txy = self._score(pts, xs, ys, ths)
            # weak prior: prefer the motion-model guess when the scan is ambiguous
            dxy = np.hypot(txy[:, 0] - pred[0], txy[:, 1] - pred[1])
            dth = np.abs(np.vectorize(wrap)(ths - pred[2]))
            sc = sc - self.prior_weight * (dxy[None, :] + 0.33 * dth[:, None])
            i, j = np.unravel_index(np.argmax(sc), sc.shape)
            best = np.array([txy[j, 0], txy[j, 1], wrap(ths[i])])
        q = float(self._score(pts, [best[0]], [best[1]], [best[2]])[0][0, 0])
        return best, q
