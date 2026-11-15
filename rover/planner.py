"""Global path planning on the SLAM map: A* + frontier search for exploration."""
import heapq
import math

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

FREE, UNKNOWN, LETHAL = 0, 1, 2
_NBRS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]


def _dilate(mask, radius_cells):
    if radius_cells <= 0:
        return mask.copy()
    if cv2 is not None:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius_cells + 1,) * 2)
        return cv2.dilate(mask.astype(np.uint8), k).astype(bool)
    out = mask.copy()
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= radius_cells * radius_cells:
                out |= np.roll(np.roll(mask, dy, 0), dx, 1)
    return out


class PlanningMap:
    """Coarse grid built from the occupancy map: state (free/unknown/lethal) + cost."""

    def __init__(self, cfg, grid, live=None):
        """live: optional Nx2 world points from the CURRENT scan (people, moved
        furniture). They block the route like walls but are never stored in the map."""
        p = cfg["planner"]
        self.res = p["resolution"]
        self.f = max(1, int(round(self.res / grid.res)))
        self.res = self.f * grid.res
        self.n = grid.n // self.f
        self.origin = grid.origin
        L = grid.L[: self.n * self.f, : self.n * self.f].reshape(self.n, self.f, self.n, self.f)
        occ = (L > 0.6).any(axis=(1, 3))
        if live is not None and len(live):
            rr = ((live[:, 1] - self.origin) // self.res).astype(int)
            cc = ((live[:, 0] - self.origin) // self.res).astype(int)
            ok = (rr >= 0) & (rr < self.n) & (cc >= 0) & (cc < self.n)
            occ = occ.copy()
            occ[rr[ok], cc[ok]] = True
        seen = (np.abs(L) > 0.2).any(axis=(1, 3))
        r = cfg["robot"]
        radius = math.hypot(r["length"] / 2, r["width"] / 2) + p["inflation"]   # corners clear walls when turning
        rc = int(math.ceil(radius / self.res))
        self.occ = occ
        self.lethal = _dilate(occ, rc)
        self.unknown = ~seen & ~self.lethal
        # soft cost: prefer staying ~0.15 m further from walls than strictly needed
        near = _dilate(occ, rc + int(math.ceil(0.15 / self.res))) & ~self.lethal
        self.cost = np.ones((self.n, self.n), dtype=np.float32)
        self.cost[near] = 4.0
        self.unknown_is_free = bool(p.get("unknown_is_free", False))

    def to_cell(self, x, y):
        return (int((y - self.origin) // self.res), int((x - self.origin) // self.res))

    def to_world(self, r, c):
        return ((c + 0.5) * self.res + self.origin, (r + 0.5) * self.res + self.origin)

    def inside(self, r, c):
        return 0 <= r < self.n and 0 <= c < self.n

    def passable(self, r, c):
        if self.lethal[r, c]:
            return False
        return self.unknown_is_free or not self.unknown[r, c]

    def nearest_passable(self, cell, max_r=6):
        r0, c0 = cell
        if self.inside(r0, c0) and self.passable(r0, c0):
            return cell
        best, bd = None, 1e9
        for dr in range(-max_r, max_r + 1):
            for dc in range(-max_r, max_r + 1):
                r, c = r0 + dr, c0 + dc
                if self.inside(r, c) and self.passable(r, c) and dr * dr + dc * dc < bd:
                    best, bd = (r, c), dr * dr + dc * dc
        return best

    def free_start(self, cell, radius=3):
        """The robot may sit inside the inflated zone (e.g. after a nudge). Let it
        plan out of there by clearing inflation (not real walls) around it."""
        r0, c0 = cell
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                r, c = r0 + dr, c0 + dc
                if self.inside(r, c) and not self.occ[r, c]:
                    self.lethal[r, c] = False
                    self.unknown[r, c] = False


def astar(pm, start, goal):
    """8-connected A*. Returns list of cells or None."""
    if not (pm.inside(*start) and pm.inside(*goal)):
        return None
    n = pm.n
    g = np.full(n * n, np.inf, dtype=np.float32)
    parent = np.full(n * n, -1, dtype=np.int32)
    closed = np.zeros(n * n, dtype=bool)
    s = start[0] * n + start[1]
    gl = goal[0] * n + goal[1]
    g[s] = 0
    gr, gc = goal
    heap = [(0.0, s)]
    cost = pm.cost
    while heap:
        _, u = heapq.heappop(heap)
        if closed[u]:
            continue
        if u == gl:
            break
        closed[u] = True
        ur, uc = divmod(u, n)
        gu = g[u]
        for dr, dc, w in _NBRS:
            r, c = ur + dr, uc + dc
            if r < 0 or r >= n or c < 0 or c >= n or not pm.passable(r, c):
                continue
            if dr and dc and not (pm.passable(ur + dr, uc) and pm.passable(ur, uc + dc)):
                continue          # no corner cutting
            v = r * n + c
            if closed[v]:
                continue
            nd = gu + w * cost[r, c]
            if nd < g[v]:
                g[v] = nd
                parent[v] = u
                h = math.hypot(r - gr, c - gc)
                heapq.heappush(heap, (nd + h, v))
    if parent[gl] < 0 and gl != s:
        return None
    path = [gl]
    while path[-1] != s:
        path.append(parent[path[-1]])
    path.reverse()
    return [divmod(int(v), n) for v in path]


def plan(cfg, grid, start_xy, goal_xy, live=None):
    """World-coordinate path from start to goal, or (None, reason).

    First tries through explored space only. If that fails (e.g. you clicked
    somewhere not yet mapped), retries treating unknown space as free; the
    path is re-planned as the map fills in while driving.
    """
    pm = PlanningMap(cfg, grid, live)
    s = pm.to_cell(*start_xy)
    if not pm.inside(*s):
        return None, "robot outside map"
    pm.free_start(s)
    reason = "no path found"
    for optimistic in (pm.unknown_is_free, True):
        pm.unknown_is_free = optimistic
        g = pm.nearest_passable(pm.to_cell(*goal_xy))
        if g is None:
            reason = "goal is inside an obstacle"
            continue
        cells = astar(pm, s, g)
        if cells is None:
            continue
        pts = [pm.to_world(r, c) for r, c in cells]
        pts[0] = tuple(start_xy)
        return _shortcut(pm, pts), ("ok" if not optimistic else "ok (through unexplored area)")
    return None, reason


def _line_clear(pm, a, b):
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    k = max(2, int(d / (pm.res * 0.5)))
    for t in np.linspace(0, 1, k):
        r, c = pm.to_cell(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        if not pm.inside(r, c) or pm.lethal[r, c] or pm.cost[r, c] > 1.0:
            return False
    return True


def _shortcut(pm, pts):
    """Remove zig-zags from the grid path where a straight line is safe."""
    if len(pts) < 3:
        return pts
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not _line_clear(pm, pts[i], pts[j]):
            j -= 1
        out.append(pts[j])
        i = j
    return _densify(out, pm.res)


def _densify(pts, step):
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        k = max(1, int(d / step))
        for t in range(1, k + 1):
            out.append((a[0] + (b[0] - a[0]) * t / k, a[1] + (b[1] - a[1]) * t / k))
    return out


def find_frontier(cfg, grid, start_xy, blacklist=(), min_dist=0.4):
    """Breadth-first search from the robot through known-free space to the nearest
    reachable frontier (free cell touching unknown). Returns world (x, y) or None."""
    pm = PlanningMap(cfg, grid)
    pm.unknown_is_free = False
    s = pm.to_cell(*start_xy)
    if not pm.inside(*s):
        return None
    pm.free_start(s)
    n = pm.n
    min_size = cfg["explore"]["min_frontier_size"]
    bl_r = cfg["explore"]["blacklist_radius"]

    def is_frontier(r, c):
        if not pm.passable(r, c):
            return False
        for dr, dc, _ in _NBRS[:4]:
            rr, cc = r + dr, c + dc
            if 0 <= rr < n and 0 <= cc < n and pm.unknown[rr, cc]:
                return True
        return False

    seen = np.zeros((n, n), dtype=bool)
    seen[s] = True
    queue = [s]
    qi = 0
    tried = np.zeros((n, n), dtype=bool)
    while qi < len(queue):
        r, c = queue[qi]
        qi += 1
        if is_frontier(r, c) and not tried[r, c]:
            cluster = _grow(pm, (r, c), is_frontier, tried)
            if len(cluster) >= min_size:
                # cell of the cluster closest to its centroid (always reachable)
                cr = np.mean([p[0] for p in cluster]); cc_ = np.mean([p[1] for p in cluster])
                tr, tc = min(cluster, key=lambda p: (p[0] - cr) ** 2 + (p[1] - cc_) ** 2)
                x, y = pm.to_world(tr, tc)
                far = math.hypot(x - start_xy[0], y - start_xy[1]) >= min_dist
                ok = all(math.hypot(x - bx, y - by) > bl_r for bx, by in blacklist)
                if far and ok:
                    return (x, y)
        for dr, dc, _ in _NBRS:
            rr, cc = r + dr, c + dc
            if 0 <= rr < n and 0 <= cc < n and not seen[rr, cc] and pm.passable(rr, cc):
                seen[rr, cc] = True
                queue.append((rr, cc))
    return None


def still_frontier(cfg, grid, xy, radius_cells=2):
    """True while there is still unexplored space next to an exploration target."""
    pm = PlanningMap(cfg, grid)
    r0, c0 = pm.to_cell(*xy)
    r_lo, r_hi = max(0, r0 - radius_cells), min(pm.n, r0 + radius_cells + 1)
    c_lo, c_hi = max(0, c0 - radius_cells), min(pm.n, c0 + radius_cells + 1)
    return bool(pm.unknown[r_lo:r_hi, c_lo:c_hi].any())


def _grow(pm, start, is_frontier, tried):
    out, stack = [], [start]
    tried[start] = True
    while stack:
        r, c = stack.pop()
        out.append((r, c))
        for dr, dc, _ in _NBRS:
            rr, cc = r + dr, c + dc
            if pm.inside(rr, cc) and not tried[rr, cc] and is_frontier(rr, cc):
                tried[rr, cc] = True
                stack.append((rr, cc))
    return out
