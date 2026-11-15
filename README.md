# Autonomous indoor vehicle

Raspberry Pi 5 · L298N · 4 × 12 V DC motors (4WD skid steer) · YDLIDAR X2 · USB webcam

Built step by step. Each step is tested on the robot before the next one starts.

| Step | What | Status |
|---|---|---|
| 1 | Motors + keyboard driving | ⏳ testing |
| 2 | Lidar readings | |
| 3 | Web dashboard + obstacle stop | |
| 4 | Room mapping | |
| 5 | Click-to-go navigation | |
| 6 | Exploration | |
| 7 | Camera stream + start on boot | |

## Setup (once, on the Pi)

```bash
git clone https://github.com/DhruvVishalPatel/autonomous_vehicle.git
cd autonomous_vehicle
./setup_venv.sh
```

After that, `git pull` gets each new step. Run every script with `./run.sh <script>`.

## Step 1: motors

Wiring: ENA→GPIO12, ENB→GPIO13, left forward/backward → GPIO5/6,
right forward/backward → GPIO21/20, L298N GND ↔ Pi GND. Pins are in `config.toml`.

```bash
./run.sh tools/test_motors.py        # wheels OFF the ground; optional speed: 0.3
./run.sh tools/drive_keyboard.py     # then on the floor: W A S D, SPACE stop, X quit
```
