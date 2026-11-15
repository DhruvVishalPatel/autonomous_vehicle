#!/usr/bin/env python3
"""Step 1 test: wheels off the ground, then run  ./run.sh tools/test_motors.py [speed]"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from rover.config import load
from rover.motors import Motors

speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
steps = [
    ("LEFT side forward", speed, 0),
    ("LEFT side backward", -speed, 0),
    ("RIGHT side forward", 0, speed),
    ("RIGHT side backward", 0, -speed),
    ("BOTH forward", speed, speed),
    ("BOTH backward", -speed, -speed),
    ("SPIN LEFT (counter-clockwise from above)", -speed, speed),
    ("SPIN RIGHT (clockwise from above)", speed, -speed),
]

m = Motors(load())
try:
    for name, l, r in steps:
        print(f"{name}  (left={l:+.2f}, right={r:+.2f})", flush=True)
        m.drive(l, r)
        time.sleep(2)
        m.stop()
        time.sleep(0.7)
    print("Done. Fix invert_left / invert_right in config.toml if a side ran backwards.")
except KeyboardInterrupt:
    print("\nstopped")
finally:
    m.close()
