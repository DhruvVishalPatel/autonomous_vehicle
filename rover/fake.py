"""Fake lidar + motors to try the dashboard on a laptop:  python3 main.py --fake"""
import math
import threading
import time

import numpy as np

from rover.lidar import Scan


class FakeMotors:
    def __init__(self, cfg):
        self.left = self.right = 0.0

    def drive(self, l, r):
        self.left, self.right = l, r

    def stop(self):
        self.left = self.right = 0.0

    def close(self):
        self.stop()


class FakeLidar:
    """3 x 4 m room with a box 0.8 m ahead."""
    def __init__(self, cfg):
        self.latest, self.rate_hz, self.on_scan = None, 7.0, None
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        a = np.linspace(-math.pi, math.pi, 450, endpoint=False)
        while True:
            c, s = np.cos(a), np.sin(a)
            with np.errstate(divide="ignore"):
                r = np.minimum(np.where(c > 0, 2.5 / c, np.where(c < 0, -1.5 / c, 9)),
                               np.where(s > 0, 1.5 / s, np.where(s < 0, -1.5 / s, 9)))
            box = (np.abs(a) < 0.18)
            r[box] = np.minimum(r[box], 0.8 / np.cos(a[box]))
            r = r + np.random.normal(0, 0.01, r.size)
            self.latest = Scan(a.copy(), r, time.monotonic())
            if self.on_scan:
                self.on_scan(self.latest)
            time.sleep(1 / 7)

    def close(self):
        pass
