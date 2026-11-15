#!/usr/bin/env python3
"""Show what the lidar returns in every 10° slice around the robot.

    ./run.sh tools/lidar_sectors.py

Robot angles: 0° = front, +90° = LEFT, -90° = RIGHT, ±180° = back
(after angle_offset_deg / flip from config.toml). Averages 10 scans.

For each slice:  raw points | valid points | nearest | median distance
'valid' = the sensor returned a distance (0 means 'no echo').
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import numpy as np
from rover.config import load
import ydlidar

c = load()["lidar"]
L = ydlidar.CYdLidar()
o = L.setlidaropt
o(ydlidar.LidarPropSerialPort, c["port"])
o(ydlidar.LidarPropSerialBaudrate, int(c["baud"]))
o(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
o(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
o(ydlidar.LidarPropScanFrequency, float(c["frequency"]))
o(ydlidar.LidarPropSampleRate, int(c["sample_rate"]))
o(ydlidar.LidarPropSingleChannel, True)
o(ydlidar.LidarPropIntenstiy, False)
o(ydlidar.LidarPropSupportMotorDtrCtrl, bool(c.get("motor_dtr", True)))
o(ydlidar.LidarPropMaxAngle, 180.0)
o(ydlidar.LidarPropMinAngle, -180.0)
o(ydlidar.LidarPropMaxRange, 16.0)
o(ydlidar.LidarPropMinRange, 0.0)
if not (L.initialize() and L.turnOn()):
    sys.exit("lidar failed to start")

offset, flip = math.radians(c.get("angle_offset_deg", 0.0)), c.get("flip", False)
A, R = [], []
scan = ydlidar.LaserScan()
try:
    time.sleep(1.0)
    got = 0
    while got < 10:
        if not L.doProcessSimple(scan) or len(scan.points) == 0:
            continue
        a = np.array([p.angle for p in scan.points])
        r = np.array([p.range for p in scan.points])
        a = -a if flip else a
        A.append((a + offset + math.pi) % (2 * math.pi) - math.pi)
        R.append(r)
        got += 1
finally:
    L.turnOff()
    L.disconnecting()

a, r = np.degrees(np.concatenate(A)), np.concatenate(R)
print(f"{'slice':>12} {'raw':>5} {'valid':>6} {'nearest':>8} {'median':>7}   (10 scans)")
for lo in range(180, -180, -10):                      # left side first, going round
    m = (a < lo) & (a >= lo - 10)
    v = m & (r > 0.02)
    near = f"{r[v].min():.2f}" if v.any() else "  -"
    med = f"{np.median(r[v]):.2f}" if v.any() else "  -"
    side = ("BACK " if abs(lo - 5) > 135 else "LEFT " if lo - 5 > 45 else
            "RIGHT" if lo - 5 < -45 else "FRONT")
    print(f"{side} {lo - 10:+4d}..{lo:+4d} {m.sum():5d} {v.sum():6d} {near:>8} {med:>7}")
