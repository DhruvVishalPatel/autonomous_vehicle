# Autonomous indoor vehicle

Raspberry Pi 5 · L298N · 4 × 12 V DC motors (4WD skid steer) · YDLIDAR X2 · USB webcam

Built step by step. Each step is tested on the robot before the next one starts.

| Step | What | Status |
|---|---|---|
| 1 | Motors + keyboard driving | ✅ done |
| 2 | Lidar readings | ✅ done |
| 3 | Web dashboard + obstacle stop | ⏳ testing |
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

## Step 2: lidar

Plug the X2's USB adapter into a blue USB 3 port on the Pi.

```bash
./setup_lidar_sdk.sh             # once; builds the YDLidar SDK into .venv if needed
./run.sh tools/test_lidar.py     # scan rate + nearest object in 8 directions
```

Expect about 7 Hz and a few hundred points per scan. Use a box to check
front / left (see the top of `tools/test_lidar.py`) and set `angle_offset_deg`
and `flip` in `config.toml`.

### Drive until an obstacle

After the box test passes:

```bash
./run.sh tools/drive_until_obstacle.py        # optional speed, e.g. 0.4
```

Drives forward, slows down within 60 cm, stops 20 cm before an obstacle (`[safety]` in `config.toml`).

### Driving straight (trim)

If the robot curves, first check with wheels off the ground that all 4 wheels spin forward
at similar speed, then tune the balance:

```bash
./run.sh tools/trim_test.py 0.05     # curves left -> raise; curves right -> lower
```

Put the value that drives straight into `config.toml` → `[motors] trim`.

## Step 3: web dashboard

```bash
./run.sh main.py
```

Open the printed address (e.g. `http://192.168.1.50:8080`) on your phone or laptop,
on the same Wi-Fi as the Pi.

- Live lidar view: robot in the middle, front is up. Red zone = stop, orange = slow down.
- Joystick (or W A S D on a laptop). Release it and the robot stops.
- STOP button (or Space) locks the motors until pressed again.
- The robot refuses to drive into anything inside the red zone, forward or reverse;
  turning on the spot is always allowed.
- Motors stop if the phone stops sending (0.5 s) or lidar data stops.

Speeds: `[drive]` in `config.toml`. Stop distances: `[safety]`.
