"""YDLIDAR X2 through the official YDLidar SDK, read in a background thread.

    lidar = Lidar(cfg)            # starts spinning and reading
    scan = lidar.latest           # newest Scan or None
    scan.angles, scan.ranges      # numpy arrays: radians (0 = front, + = left), metres
    lidar.close()
"""
import math
import threading
import time

import numpy as np


class Scan:
    __slots__ = ("angles", "ranges", "stamp")

    def __init__(self, angles, ranges, stamp):
        self.angles, self.ranges, self.stamp = angles, ranges, stamp

    def points(self):
        """Nx2 array of (x forward, y left) in metres."""
        return np.column_stack((self.ranges * np.cos(self.angles),
                                self.ranges * np.sin(self.angles)))


class Lidar:
    def __init__(self, cfg, on_scan=None):
        import ydlidar

        c = cfg["lidar"]
        self.yd = ydlidar
        self.offset = math.radians(c.get("angle_offset_deg", 0.0))
        self.flip = bool(c.get("flip", False))
        self.rmin, self.rmax = c["min_range"], c["max_range"]
        self.on_scan = on_scan
        self.latest = None
        self.rate_hz = 0.0
        self.scans = 0

        laser = ydlidar.CYdLidar()
        o = laser.setlidaropt
        o(ydlidar.LidarPropSerialPort, c["port"])
        o(ydlidar.LidarPropSerialBaudrate, int(c["baud"]))
        o(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        o(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        o(ydlidar.LidarPropScanFrequency, float(c["frequency"]))
        o(ydlidar.LidarPropSampleRate, int(c["sample_rate"]))
        o(ydlidar.LidarPropSingleChannel, True)
        o(ydlidar.LidarPropIntenstiy, False)          # (sic) SDK spelling
        o(ydlidar.LidarPropSupportMotorDtrCtrl, bool(c.get("motor_dtr", True)))
        o(ydlidar.LidarPropAutoReconnect, True)
        o(ydlidar.LidarPropMaxAngle, 180.0)
        o(ydlidar.LidarPropMinAngle, -180.0)
        o(ydlidar.LidarPropMaxRange, float(c["max_range"]))
        o(ydlidar.LidarPropMinRange, float(c["min_range"]))
        if not laser.initialize():
            raise RuntimeError(f"lidar initialize() failed on {c['port']} — check cable/port/permissions")
        if not laser.turnOn():
            laser.disconnecting()
            raise RuntimeError("lidar turnOn() failed")
        self.laser = laser
        self._run = True
        self._t = threading.Thread(target=self._loop, name="lidar", daemon=True)
        self._t.start()

    def _loop(self):
        scan = self.yd.LaserScan()
        last = None
        while self._run:
            if not self.laser.doProcessSimple(scan):
                time.sleep(0.02)
                continue
            n = len(scan.points)
            if n == 0:
                continue
            ang = np.fromiter((p.angle for p in scan.points), float, n)
            rng = np.fromiter((p.range for p in scan.points), float, n)
            if self.flip:
                ang = -ang
            ang = (ang + self.offset + math.pi) % (2 * math.pi) - math.pi
            ok = (rng >= self.rmin) & (rng <= self.rmax)
            order = np.argsort(ang[ok])
            s = Scan(ang[ok][order], rng[ok][order], time.monotonic())
            if last is not None:
                self.rate_hz = 0.8 * self.rate_hz + 0.2 / max(s.stamp - last, 1e-3)
            last = s.stamp
            self.latest = s
            self.scans += 1
            if self.on_scan:
                self.on_scan(s)

    def close(self):
        self._run = False
        self._t.join(timeout=1.0)
        try:
            self.laser.turnOff()
            self.laser.disconnecting()
        except Exception:
            pass
