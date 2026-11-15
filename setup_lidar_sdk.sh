#!/usr/bin/env bash
# Step 2: make the YDLidar SDK (python module 'ydlidar') importable in .venv.
# Skips the build if it already works. Safe to re-run.
set -euo pipefail
cd "$(dirname "$0")"
PY=.venv/bin/python
[ -x "$PY" ] || { echo "Run ./setup_venv.sh first"; exit 1; }

if "$PY" -c "import ydlidar" 2>/dev/null; then
  echo "ydlidar already importable in .venv — nothing to do"
else
  echo "==> building YDLidar-SDK into .venv (takes a few minutes)"
  sudo apt install -y git cmake swig build-essential python3-dev
  if [ ! -d third_party/YDLidar-SDK ]; then
    mkdir -p third_party
    git clone --depth 1 https://github.com/YDLIDAR/YDLidar-SDK.git third_party/YDLidar-SDK
  fi
  "$PY" -m pip install ./third_party/YDLidar-SDK
fi
"$PY" -c "import ydlidar; print('ydlidar OK')"

# serial port access for the lidar
if ! id -nG "$USER" | grep -qw dialout; then
  sudo usermod -aG dialout "$USER"
  echo "Added $USER to 'dialout' — log out and back in (or reboot) once."
fi
ls -l /dev/ttyUSB* 2>/dev/null || echo "No /dev/ttyUSB* found — is the lidar plugged in?"
