#!/usr/bin/env python3
"""Step 2 test:  ./run.sh tools/test_lidar.py

Prints scan rate and the nearest object in 8 directions twice a second.

Orientation check:
 1. Put a box ~40 cm straight in FRONT of the robot -> 'front' should drop to ~0.4.
    If it shows up in another sector, set angle_offset_deg in config.toml
    (box shows under 'left' -> -90, 'right' -> 90, 'back' -> 180).
 2. Move the box to the robot's LEFT -> 'left' should drop.
    If 'right' drops instead, set flip = true.
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import numpy as np
from rover.config import load
from rover.lidar import Lidar

NAMES = ["front", "f-left", "left", "b-left", "back", "b-right", "right", "f-right"]

lidar = Lidar(load())
print("lidar started, waiting for scans… (Ctrl+C to stop)")
try:
    t0 = time.monotonic()
    while True:
        time.sleep(0.5)
        s = lidar.latest
        if s is None:
            if time.monotonic() - t0 > 5:
                print("no scans after 5 s — is the lidar spinning? check port in config.toml")
                t0 = time.monotonic()
            continue
        sector = (((s.angles + math.pi / 8) % (2 * math.pi)) // (math.pi / 4)).astype(int)
        cells = []
        for i, name in enumerate(NAMES):
            r = s.ranges[sector == i]
            cells.append(f"{name}:{r.min():4.2f}" if r.size else f"{name}: -- ")
        k = int(np.argmin(s.ranges))
        print(f"{lidar.rate_hz:4.1f} Hz {len(s.ranges):3d} pts | nearest {s.ranges[k]:.2f} m at "
              f"{math.degrees(s.angles[k]):+4.0f}° | " + " ".join(cells), flush=True)
except KeyboardInterrupt:
    pass
finally:
    lidar.close()
    print("lidar stopped")
