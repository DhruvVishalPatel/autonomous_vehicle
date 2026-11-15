#!/usr/bin/env python3
"""Drive the robot from the keyboard over SSH:  ./run.sh tools/drive_keyboard.py

  W / S      forward / backward          A / D   turn left / right
  Q / E      curve forward-left / -right SPACE   stop
  + / -      speed up / down             X       quit
Hold a key to keep moving. Release it and the robot stops (dead-man).
"""
import curses
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from rover.config import load
from rover.motors import Motors

# key -> (left, right) direction
KEYS = {
    ord("w"): (1, 1), ord("s"): (-1, -1),
    ord("a"): (-1, 1), ord("d"): (1, -1),
    ord("q"): (0.4, 1), ord("e"): (1, 0.4),
    curses.KEY_UP: (1, 1), curses.KEY_DOWN: (-1, -1),
    curses.KEY_LEFT: (-1, 1), curses.KEY_RIGHT: (1, -1),
}
HOLD = 0.6   # s; covers the keyboard's auto-repeat delay


def main(scr):
    curses.curs_set(0)
    scr.nodelay(True)
    scr.keypad(True)
    m = Motors(load())
    speed, cmd, last = 0.5, (0, 0), 0.0
    try:
        while True:
            k = scr.getch()
            now = time.monotonic()
            if k in (ord("x"), ord("X")):
                break
            if k == ord(" "):
                cmd, last = (0, 0), 0.0
            elif k in (ord("+"), ord("=")):
                speed = min(1.0, speed + 0.1)
            elif k in (ord("-"), ord("_")):
                speed = max(0.2, speed - 0.1)
            elif k != -1 and (k | 0x20 if k < 256 else k) in KEYS:
                cmd, last = KEYS[k | 0x20 if k < 256 else k], now
            if now - last > HOLD:
                cmd = (0, 0)
            m.drive(cmd[0] * speed, cmd[1] * speed)
            scr.erase()
            scr.addstr(0, 0, __doc__.strip().splitlines()[0])
            scr.addstr(2, 0, "W/S fwd/back  A/D turn  Q/E curve  SPACE stop  +/- speed  X quit")
            scr.addstr(4, 0, f"speed {speed:.1f}   left {cmd[0] * speed:+.2f}   right {cmd[1] * speed:+.2f}")
            scr.refresh()
            time.sleep(0.03)
    finally:
        m.close()


if __name__ == "__main__":
    curses.wrapper(main)
