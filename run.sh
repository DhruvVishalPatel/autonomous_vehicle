#!/usr/bin/env bash
# Run any script with the project venv:  ./run.sh tools/test_motors.py
set -e
cd "$(dirname "$0")"
[ -x .venv/bin/python ] || { echo "Run ./setup_venv.sh first"; exit 1; }
exec .venv/bin/python "$@"
