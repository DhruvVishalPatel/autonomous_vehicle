"""Go-to-goal: plan a route on the map with A*, then follow it.

Two halves:
* a planning thread (~5 Hz) that runs A* and replans every few seconds
* command(), called by the 20 Hz control loop, which steers along the path

Steering uses the map position only (no speed calibration needed): aim at a
point `lookahead` metres ahead on the path, turn in proportion to the heading
error, spin on the spot first if the target is far off to the side.
"""
import math
import threading
import time

import numpy as np

from rover import planner
from rover.slam import wrap


class Navigator:
    def __init__(self, cfg, slam, log=print):
        self.cfg, self.slam, self.log = cfg, slam, log
        n = cfg["nav"]
        self.speed, self.wmax, self.k = n["speed"], n["turn_speed"], n["turn_gain"]
        self.turn_first = n["turn_in_place"]
        self.lookahead, self.tol = n["lookahead"], n["goal_tolerance"]
        self.replan_period, self.stuck_time = n["replan_period"], n["stuck_time"]
        self.goal = None          # (x, y)
        self.path = None          # [(x, y), ...]
        self.status = "idle"
        self._need_plan = False
        self._last_plan = 0.0
        self._idx = 0
        self._recovery = []       # [(duration, v, w), ...]
        self._phase_end = 0.0
        self._recoveries = 0
        self._hist = []           # (t, x, y, th) while trying to move
        self._pose = np.zeros(3)
        self._lock = threading.Lock()
        threading.Thread(target=self._plan_loop, name="planner", daemon=True).start()

    # ------------------------------------------------------------------ API
    @property
    def active(self):
        return self.goal is not None

    def go_to(self, x, y):
        with self._lock:
            self.goal, self.path = (float(x), float(y)), None
            self._need_plan, self._recovery, self._recoveries = True, [], 0
            self._hist.clear()
        self._set_status(f"planning to ({x:.2f}, {y:.2f})")

    def cancel(self, why="cancelled"):
        with self._lock:
            self.goal, self.path, self._recovery = None, None, []
        self._set_status(why)

    def _set_status(self, s):
        if s != self.status:
            self.status = s
            self.log(s)

    # -------------------------------------------------------- planning thread
    def _plan_loop(self):
        while True:
            time.sleep(0.2)
            try:
                self._plan_tick()
            except Exception as e:
                self.cancel(f"planner error: {e}")

    def _plan_tick(self):
        goal = self.goal
        if goal is None or self._recovery:
            return
        if not self._need_plan and time.monotonic() - self._last_plan < self.replan_period:
            return
        path, reason = planner.plan(self.cfg, self.slam.grid, self._pose[:2], goal)
        with self._lock:
            if self.goal != goal:          # goal changed meanwhile
                return
            self._last_plan, self._need_plan = time.monotonic(), False
            if path is None:
                self.goal, self.path = None, None
                self._set_status(f"failed: {reason}")
                return
            first = self.path is None
            self.path, self._idx = path, 0
        if first:
            self._set_status(f"driving to ({goal[0]:.2f}, {goal[1]:.2f})"
                             + (" through unexplored area" if "unexplored" in reason else ""))

    # ----------------------------------------------------------- control tick
    def command(self, pose, safety):
        """(v, w) motor-command units for this tick; (0, 0) when idle."""
        self._pose = pose.copy()
        now = time.monotonic()
        if self.goal is None:
            return 0.0, 0.0

        if self._recovery:                       # back up / turn manoeuvre
            _, v, w = self._recovery[0]
            if now < self._phase_end:
                return v, w
            self._recovery.pop(0)
            if self._recovery:
                self._phase_end = now + self._recovery[0][0]
            else:
                self._need_plan = True
                self._hist.clear()
            return 0.0, 0.0

        path = self.path
        if not path:
            return 0.0, 0.0                      # waiting for the first plan

        gx, gy = path[-1]
        if math.hypot(gx - pose[0], gy - pose[1]) < self.tol:
            self.cancel("goal reached")
            return 0.0, 0.0

        if safety.blocked_for() > 1.5 or self._stuck(now, pose):
            self._start_recovery(safety)
            return 0.0, 0.0

        v, w = self._steer(pose, path)
        self._hist.append((now, pose[0], pose[1], pose[2]))
        return v, w

    def _steer(self, pose, path):
        best_i, best_d = self._idx, 1e9          # progress along the path, never backwards
        for i in range(self._idx, min(len(path), self._idx + 40)):
            d = math.hypot(path[i][0] - pose[0], path[i][1] - pose[1])
            if d < best_d:
                best_i, best_d = i, d
        self._idx = best_i
        if best_d > 0.6:
            self._need_plan = True               # drifted off the path
        target = path[-1]
        for i in range(best_i, len(path)):
            if math.hypot(path[i][0] - pose[0], path[i][1] - pose[1]) >= self.lookahead:
                target = path[i]
                break
        err = wrap(math.atan2(target[1] - pose[1], target[0] - pose[0]) - pose[2])
        w = max(-self.wmax, min(self.wmax, self.k * err))
        if abs(err) > self.turn_first:
            return 0.0, w                        # spin on the spot towards the path
        dist_goal = math.hypot(path[-1][0] - pose[0], path[-1][1] - pose[1])
        v = self.speed * max(0.3, math.cos(err)) * min(1.0, 0.4 + dist_goal / 0.5)
        return v, w

    def _stuck(self, now, pose):
        h = self._hist
        while h and now - h[0][0] > self.stuck_time:
            h.pop(0)
        if len(h) < 20 or now - h[0][0] < 0.9 * self.stuck_time:
            return False
        moved = math.hypot(pose[0] - h[0][1], pose[1] - h[0][2])
        turned = abs(wrap(pose[2] - h[0][3]))
        return moved < 0.05 and turned < 0.15

    def _start_recovery(self, safety):
        self._recoveries += 1
        self._hist.clear()
        if self._recoveries > 4:
            self.cancel("failed: blocked too many times")
            return
        phases = []
        if safety.rear > 0.15:
            phases.append((1.0, -0.35, 0.0))     # back up
        phases.append((0.8, 0.0, self.wmax))     # turn a bit
        self._recovery = phases
        self._phase_end = time.monotonic() + phases[0][0]
        self._set_status("blocked — backing up and replanning")
