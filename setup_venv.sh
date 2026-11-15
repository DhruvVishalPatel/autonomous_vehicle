#!/usr/bin/env bash
# One-time setup on the Pi (Raspberry Pi OS Trixie). Safe to re-run.
set -euo pipefail
cd "$(dirname "$0")"
sudo apt install -y python3-venv python3-gpiozero python3-lgpio python3-numpy
[ -x .venv/bin/python ] || python3 -m venv --system-site-packages .venv
.venv/bin/pip install -q -r requirements.txt
.venv/bin/python -c "import gpiozero, lgpio; print('venv OK: gpiozero', gpiozero.__version__)"
