#!/usr/bin/env python3
"""Time each part of the mapping on the Pi.   ./run.sh tools/slam_bench.py

Stop main.py first. Keep the robot still. Grabs 15 lidar scans, then runs
the mapping on them and prints how long each part takes.
"""
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import numpy as np
from rover.config import load
from rover import slam as slam_mod
from rover.lidar import Lidar

print("python", platform.python_version(), "| numpy", np.__version__,
      "| opencv", getattr(slam_mod.cv2, "__version__", "MISSING"), "| cpus", os.cpu_count())
for cmd in (["vcgencmd", "measure_temp"], ["vcgencmd", "get_throttled"]):
    try:
        print(" ", subprocess.run(cmd, capture_output=True, text=True, timeout=3).stdout.strip())
    except Exception:
        pass
try:
    print("  load average:", os.getloadavg())
except OSError:
    pass

# raw numpy speed check
t = time.perf_counter()
a = np.random.rand(1000, 1000); (a @ a).sum()
print(f"  numpy 1000x1000 matmul: {(time.perf_counter() - t) * 1000:.0f} ms  (Pi 5 normal: ~100-300 ms)")

cfg = load()
lidar = Lidar(cfg)
scans, last = [], None
print("collecting 15 scans…")
t0 = time.monotonic()
while len(scans) < 15 and time.monotonic() - t0 < 20:
    s = lidar.latest
    if s is not None and s is not last:
        scans.append(s)
        last = s
    time.sleep(0.02)
lidar.close()
print(f"got {len(scans)} scans, {np.mean([len(s.ranges) for s in scans]):.0f} points each")

sl = slam_mod.SLAM(cfg)
print(f"\n{'scan':>4} {'field':>8} {'match':>8} {'map_upd':>8} {'total':>8}  (ms)")
for i, s in enumerate(scans):
    # pretend the robot moved 2 cm each scan so the map gets updated too
    sl.process(s, np.array([0.02, 0, 0]) if i else np.zeros(3))
    tm = sl.timing
    print(f"{i:4d} {tm.get('field', 0):8.1f} {tm.get('match', 0):8.1f} "
          f"{tm.get('map_update', 0):8.1f} {tm.get('total', 0):8.1f}")
