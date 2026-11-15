#!/usr/bin/env python3
"""Lidar diagnostics: are we getting full, stable revolutions?

    ./run.sh tools/lidar_diag.py                       # settings from config.toml
    ./run.sh tools/lidar_diag.py --freq 6 --rate 3     # try other SDK settings
    ./run.sh tools/lidar_diag.py --no-single           # try single_channel = false
    ./run.sh tools/lidar_diag.py --no-dtr              # SDK doesn't touch the motor line

Keep the robot still. Collects 40 scans straight from the SDK and reports:
  rate      scans per second (X2 should be ~6-8 Hz)
  points    raw points per scan, and how many have a valid distance
  coverage  % of the 360° circle that has at least one valid point (1° bins)
  span      first..last angle in the scan (a full turn is about -180..+180)
  gap       largest angle with no points at all
  repeat    how many points are still there in the next scan (stability)
"""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import numpy as np
from rover.config import load
import ydlidar

ap = argparse.ArgumentParser()
ap.add_argument("--freq", type=float)
ap.add_argument("--rate", type=int, help="sample rate in kHz")
ap.add_argument("--no-single", action="store_true")
ap.add_argument("--no-dtr", action="store_true", help="do not let the SDK drive the motor via DTR")
ap.add_argument("--n", type=int, default=40)
a = ap.parse_args()

c = load()["lidar"]
freq = a.freq if a.freq is not None else c["frequency"]
rate = a.rate if a.rate is not None else c["sample_rate"]
single = not a.no_single
dtr = not a.no_dtr and c.get("motor_dtr", True)
print(f"port {c['port']}  baud {c['baud']}  freq {freq}  sample_rate {rate}  single_channel {single}  motor_dtr {dtr}")

L = ydlidar.CYdLidar()
o = L.setlidaropt
o(ydlidar.LidarPropSerialPort, c["port"])
o(ydlidar.LidarPropSerialBaudrate, int(c["baud"]))
o(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
o(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
o(ydlidar.LidarPropScanFrequency, float(freq))
o(ydlidar.LidarPropSampleRate, int(rate))
o(ydlidar.LidarPropSingleChannel, single)
o(ydlidar.LidarPropIntenstiy, False)
o(ydlidar.LidarPropSupportMotorDtrCtrl, bool(dtr))
o(ydlidar.LidarPropMaxAngle, 180.0)
o(ydlidar.LidarPropMinAngle, -180.0)
o(ydlidar.LidarPropMaxRange, 16.0)      # wide limits: see everything the sensor sends
o(ydlidar.LidarPropMinRange, 0.01)
if not (L.initialize() and L.turnOn()):
    sys.exit("lidar failed to start")

scan = ydlidar.LaserScan()
rows, prev_bins, t_prev = [], None, None
try:
    time.sleep(1.0)                          # let the motor settle
    while len(rows) < a.n:
        if not L.doProcessSimple(scan):
            continue
        t = time.monotonic()
        n = len(scan.points)
        if n == 0:
            continue
        ang = np.degrees(np.fromiter((p.angle for p in scan.points), float, n))
        rng = np.fromiter((p.range for p in scan.points), float, n)
        ok = rng > 0.02
        bins = np.zeros(360, bool)
        bins[((ang[ok] + 180) % 360).astype(int)] = True
        srt = np.sort(ang)
        gaps = np.diff(np.concatenate((srt, [srt[0] + 360])))
        repeat = (bins & prev_bins).sum() / max(bins.sum(), 1) * 100 if prev_bins is not None else float("nan")
        dt = t - t_prev if t_prev else float("nan")
        rows.append((dt, n, ok.sum(), bins.mean() * 100, srt[0], srt[-1], gaps.max(), repeat))
        prev_bins, t_prev = bins, t
        print(f"{len(rows):3d}  dt {dt * 1000:6.1f} ms  pts {n:4d}  valid {ok.sum():4d}  "
              f"coverage {bins.mean() * 100:5.1f}%  span {srt[0]:+7.1f}..{srt[-1]:+7.1f}  "
              f"gap {gaps.max():5.1f}°  repeat {repeat:5.1f}%", flush=True)
finally:
    L.turnOff()
    L.disconnecting()

r = np.array(rows[1:], float)
print("\nSUMMARY")
print(f"  rate      {1 / np.nanmean(r[:, 0]):.1f} Hz   (dt spread {np.nanstd(r[:, 0]) * 1000:.0f} ms)")
print(f"  points    {r[:, 1].mean():.0f} raw / {r[:, 2].mean():.0f} valid per scan")
print(f"  coverage  {r[:, 3].mean():.0f}%   largest gap {r[:, 6].mean():.0f}°   repeat {np.nanmean(r[:, 7]):.0f}%")
full = r[:, 3].mean() > 60 and r[:, 6].mean() < 40
hz = 1 / np.nanmean(r[:, 0])
print(f"  points/turn at this speed: {r[:, 1].mean():.0f}  (X2 ~430 at 7 Hz)")
print("  verdict  ", "spinning too fast" if hz > 9 else "speed OK", "/",
      "full turns" if r[:, 4].mean() < -170 and r[:, 5].mean() > 165 else "PARTIAL turns")
