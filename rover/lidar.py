"""YDLIDAR X2 through the official YDLidar SDK.

The SDK runs in its OWN PROCESS: its blocking read holds Python's GIL, which
would starve mapping and the dashboard if it ran as a thread in the main program.
Finished scans are sent back over a pipe.

    lidar = Lidar(cfg)            # starts spinning and reading
    scan = lidar.latest           # newest Scan or None
    scan.angles, scan.ranges      # numpy arrays: radians (0 = front, + = left), metres
    lidar.close()
"""
import math
import multiprocessing as mp
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


def _reader(c, conn, stop):
    """Runs in the child process: own the SDK, send (angles, ranges, stamp) per turn."""
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)     # Ctrl+C goes to the main program, which stops us
    try:
        import ydlidar

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
            conn.send(("error", f"lidar initialize() failed on {c['port']} — check cable/port/permissions"))
            return
        if not laser.turnOn():
            laser.disconnecting()
            conn.send(("error", "lidar turnOn() failed"))
            return
        conn.send(("ready", None))
        offset = math.radians(c.get("angle_offset_deg", 0.0))
        flip = bool(c.get("flip", False))
        rmin, rmax = c["min_range"], c["max_range"]
        scan = ydlidar.LaserScan()
        while not stop.is_set():
            if not laser.doProcessSimple(scan):
                time.sleep(0.01)
                continue
            n = len(scan.points)
            if n == 0:
                continue
            ang = np.fromiter((p.angle for p in scan.points), float, n)
            rng = np.fromiter((p.range for p in scan.points), float, n)
            if flip:
                ang = -ang
            ang = (ang + offset + math.pi) % (2 * math.pi) - math.pi
            ok = (rng >= rmin) & (rng <= rmax)
            order = np.argsort(ang[ok])
            conn.send(("scan", (ang[ok][order], rng[ok][order], time.monotonic())))
        laser.turnOff()
        laser.disconnecting()
    except Exception as e:                      # report instead of dying silently
        try:
            conn.send(("error", f"lidar process: {e}"))
        except Exception:
            pass


class Lidar:
    def __init__(self, cfg, on_scan=None):
        self.on_scan = on_scan
        self.latest = None
        self.rate_hz = 0.0
        self.scans = 0
        ctx = mp.get_context("fork")      # start the lidar BEFORE other threads
        self._conn, child = ctx.Pipe(duplex=False)
        self._stop = ctx.Event()
        self._proc = ctx.Process(target=_reader, args=(cfg["lidar"], child, self._stop),
                                 name="lidar", daemon=True)
        self._proc.start()
        if not self._conn.poll(15):
            self.close()
            raise RuntimeError("lidar process did not start within 15 s")
        kind, msg = self._conn.recv()
        if kind == "error":
            self.close()
            raise RuntimeError(msg)
        self._run = True
        self._t = threading.Thread(target=self._loop, name="lidar-recv", daemon=True)
        self._t.start()

    def _loop(self):
        last = None
        while self._run:
            try:
                if not self._conn.poll(0.5):
                    continue
                kind, data = self._conn.recv()
            except (EOFError, OSError):
                print("[lidar] process ended")
                break
            if kind == "error":
                print(f"[lidar] {data}")
                continue
            # only keep the newest scan if several queued up
            while self._conn.poll():
                try:
                    k2, d2 = self._conn.recv()
                    if k2 == "scan":
                        data = d2
                except (EOFError, OSError):
                    break
            s = Scan(*data)
            if last is not None:
                self.rate_hz = 0.8 * self.rate_hz + 0.2 / max(s.stamp - last, 1e-3)
            last = s.stamp
            self.latest = s
            self.scans += 1
            if self.on_scan:
                self.on_scan(s)

    def close(self):
        self._run = False
        self._stop.set()
        self._proc.join(timeout=3)
        if self._proc.is_alive():
            self._proc.terminate()
