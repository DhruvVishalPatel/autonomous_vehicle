"""Ties lidar + motors + safety together with a 20 Hz control loop."""
import math
import threading
import time

from rover.safety import Safety


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

    # ---------------------------------------------------------- inputs
    def joystick(self, x, y):
        """x: -1 (left) .. 1 (right), y: -1 (back) .. 1 (forward)."""
        x, y = max(-1.0, min(1.0, float(x))), max(-1.0, min(1.0, float(y)))
        self._joy = (y * self.vmax, -x * self.wmax, time.monotonic())

    def set_estop(self, on):
        self.estop = bool(on)
        if on:
            self.motors.stop()

    # ---------------------------------------------------------- loop
    def _loop(self):
        dt = 0.05
        while self._run:
            t0 = time.monotonic()
            v, w, tj = self._joy
            scan = self.lidar.latest
            if self.estop:
                v = w = 0.0
                self.status = "E-STOP"
            elif scan is None or t0 - scan.stamp > self.lidar_timeout:
                v = w = 0.0
                self.status = "no lidar data"
            elif t0 - tj > self.manual_timeout:
                v = w = 0.0
                self.safety.filter(0, 0, scan)        # keep clearances updated
                self.status = "idle"
            else:
                v, w = self.safety.filter(v, w, scan)
                self.status = self.safety.reason or "driving"
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
        return {
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
