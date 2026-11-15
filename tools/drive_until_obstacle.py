#!/usr/bin/env python3
"""Drive forward until something is in front, then stop.

    ./run.sh tools/drive_until_obstacle.py [speed]      (default speed 0.5)

Only run this AFTER test_lidar.py shows a box in front as 'front'
(otherwise the robot is watching the wrong direction).

Stops when:
  * an obstacle is within safety.stop_distance of the front edge
  * lidar data stops arriving (> 0.5 s old)
  * 20 s have passed, or you press Ctrl+C
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import numpy as np
from rover.config import load
from rover.lidar import Lidar
from rover.motors import Motors

cfg = load()
speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
half_len = cfg["robot"]["length"] / 2
half_wid = cfg["robot"]["width"] / 2 + 0.03          # 3 cm side margin
stop_d = cfg["safety"]["stop_distance"]
slow_d = cfg["safety"]["slow_distance"]
MAX_TIME, MIN_SPEED = 20.0, 0.30


def front_clearance(scan):
    """Distance from the robot's front edge to the nearest point in its path."""
    x, y = scan.points().T
    ahead = (x > 0) & (np.abs(y) < half_wid)
    return float(x[ahead].min() - half_len) if ahead.any() else float("inf")


lidar = Lidar(cfg)
motors = Motors(cfg)
try:
    print("waiting for lidar…")
    t0 = time.monotonic()
    while lidar.latest is None:
        if time.monotonic() - t0 > 8:
            sys.exit("no lidar data — run tools/test_lidar.py first")
        time.sleep(0.1)

    start = time.monotonic()
    reason = "time limit"
    while time.monotonic() - start < MAX_TIME:
        scan = lidar.latest
        if time.monotonic() - scan.stamp > 0.5:
            reason = "lidar data stopped"
            break
        clear = front_clearance(scan)
        if clear <= stop_d:
            reason = f"obstacle {clear:.2f} m ahead"
            break
        # full speed when far, slow down linearly inside slow_distance
        k = 1.0 if clear >= slow_d else max((clear - stop_d) / (slow_d - stop_d), 0.0)
        v = max(MIN_SPEED, speed * k) if k > 0 else 0.0
        motors.drive(v, v)
        print(f"clearance {clear:5.2f} m   speed {v:.2f}", flush=True)
        time.sleep(0.05)
    motors.stop()
    print(f"STOPPED: {reason}")
except KeyboardInterrupt:
    print("\nSTOPPED: Ctrl+C")
finally:
    motors.close()
    lidar.close()
