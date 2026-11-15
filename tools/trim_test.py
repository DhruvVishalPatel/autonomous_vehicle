#!/usr/bin/env python3
"""Find the trim value that makes the robot drive straight.

    ./run.sh tools/trim_test.py TRIM [SPEED] [SECONDS]
    e.g. ./run.sh tools/trim_test.py 0.05          (speed 0.5, 3 s)

Put a strip of tape on the floor, start the robot on it, run, and watch:
  still curves LEFT  -> try a bigger trim  (0.05 -> 0.10)
  now curves RIGHT   -> try a smaller trim (0.10 -> 0.07)
When it stays on the tape, put that value in config.toml -> [motors] trim.
Stops early if something is within 25 cm in front (lidar), or on Ctrl+C.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import numpy as np
from rover.config import load
from rover.motors import Motors

cfg = load()
trim = float(sys.argv[1]) if len(sys.argv) > 1 else cfg["motors"].get("trim", 0.0)
speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
secs = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0

lidar = None
try:
    from rover.lidar import Lidar
    lidar = Lidar(cfg)
except Exception as e:
    print(f"(lidar not available: {e} — no obstacle stop, keep your hand on Ctrl+C)")

m = Motors(cfg)
m.set_trim(trim)
half_len, half_wid = cfg["robot"]["length"] / 2, cfg["robot"]["width"] / 2 + 0.03
print(f"trim {trim:+.3f}  speed {speed}  for {secs} s  (left x{m.k_l:.2f}, right x{m.k_r:.2f})")
try:
    time.sleep(1.0 if lidar else 0.0)
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        s = lidar.latest if lidar else None
        if s is not None:
            x, y = s.points().T
            ahead = (x > 0) & (np.abs(y) < half_wid)
            if ahead.any() and x[ahead].min() - half_len < 0.25:
                print("stopped: obstacle ahead")
                break
        m.drive(speed, speed)
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nstopped")
finally:
    m.close()
    if lidar:
        lidar.close()
