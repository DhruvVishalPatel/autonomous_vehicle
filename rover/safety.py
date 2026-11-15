"""Obstacle stop. Every motor command passes through Safety.filter()."""
import math

import numpy as np


class Safety:
    def __init__(self, cfg):
        self.half_len = cfg["robot"]["length"] / 2
        self.half_wid = cfg["robot"]["width"] / 2 + 0.03
        self.stop_d = cfg["safety"]["stop_distance"]
        self.slow_d = cfg["safety"]["slow_distance"]
        self.front = self.rear = math.inf
        self.reason = ""

    def clearances(self, scan):
        """Distance from the front / rear edge to the nearest point in the robot's path."""
        x, y = scan.points().T
        lane = np.abs(y) < self.half_wid
        f, b = lane & (x > 0), lane & (x < 0)
        front = float(x[f].min() - self.half_len) if f.any() else math.inf
        rear = float(-x[b].max() - self.half_len) if b.any() else math.inf
        return front, rear

    def _scale(self, clear):
        if clear <= self.stop_d:
            return 0.0
        if clear >= self.slow_d:
            return 1.0
        return max(0.3, (clear - self.stop_d) / (self.slow_d - self.stop_d))

    def filter(self, v, w, scan):
        """v: forward (-1..1), w: turn left (-1..1). Returns safe (v, w)."""
        self.front, self.rear = self.clearances(scan)
        self.reason = ""
        if v > 0:
            k = self._scale(self.front)
            if k == 0:
                self.reason = f"obstacle ahead ({self.front:.2f} m)"
            v *= k
        elif v < 0:
            k = self._scale(self.rear)
            if k == 0:
                self.reason = f"obstacle behind ({self.rear:.2f} m)"
            v *= k
        return v, w            # turning on the spot is always allowed
