#!/usr/bin/env python3
"""Start the robot + dashboard.   ./run.sh main.py        (laptop test: python3 main.py --fake)"""
import argparse
import signal
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rover.config import load
from rover.robot import Robot
from rover.web.server import serve


def ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "localhost"


ap = argparse.ArgumentParser()
ap.add_argument("--fake", action="store_true", help="fake lidar + motors (no hardware)")
args = ap.parse_args()
cfg = load()
robot = Robot(cfg, fake=args.fake)
srv = serve(robot, cfg["web"]["port"])
print(f"Dashboard: http://{ip()}:{cfg['web']['port']}   (Ctrl+C to stop)", flush=True)

stop = []
signal.signal(signal.SIGTERM, lambda *a: stop.append(1))
try:
    while not stop:
        time.sleep(0.2)
except KeyboardInterrupt:
    pass
finally:
    srv.shutdown()
    robot.close()
    print("stopped")
