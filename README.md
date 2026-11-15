# Autonomous indoor vehicle

Raspberry Pi 5 · L298N · 4 × 12 V DC motors (4WD skid steer) · YDLIDAR X2 · USB webcam

A web dashboard (phone or laptop) to **drive**, **map a room**, **click a point to go there**
(steering around people) and **explore automatically**, with a live camera feed.
Position is estimated with an **Unscented Kalman Filter** fusing the motor commands with
lidar scan matching.

| Step | What | Status |
|---|---|---|
| 1 | Motors + keyboard driving | ✅ |
| 2 | Lidar readings | ✅ |
| 3 | Web dashboard + obstacle stop | ✅ |
| 4 | Room mapping | ✅ |
| 5 | Click-to-go + avoiding people | ✅ |
| 6 | Exploration + UKF localisation | ⏳ testing |
| 7 | Camera feed + final UI | ⏳ testing |

## Setup (once, on the Pi)

```bash
git clone https://github.com/DhruvVishalPatel/autonomous_vehicle.git
cd autonomous_vehicle
./setup_venv.sh          # apt packages (incl. OpenCV) + .venv
./setup_lidar_sdk.sh     # YDLidar SDK into .venv
```

After that, `git pull` gets updates. Run everything with `./run.sh <script>`.

## Run

```bash
./run.sh main.py
```

Open the printed address (e.g. `http://172.20.10.3:8080`) on a phone or laptop on the same Wi-Fi.

| Mode | How |
|---|---|
| **Drive** | Joystick, or W A S D. Release = stop. |
| **Go to** | Click anywhere on the map. Blue line = route, green dot = goal. |
| **Explore** | Drives to unexplored edges until the area is mapped, then stops. |
| **STOP** | Big red button or Space. Locks the motors until released. |

Touching the joystick always takes over from Go to / Explore.
**Map / Lidar** (top left) switches between the map and a robot-centred lidar view with the stop zones.
**Save / Load / Clear** the map in the Map card (Load expects the robot back at that map's start spot).

## How it works

```
 lidar process ──► scan ──► scan matcher ──► UKF update ──► fused pose ──► map + navigation
 (YDLidar SDK)                 ▲                ▲                 │
                               └── UKF predict ─┘◄── wheel commands (20 Hz)
```

- **UKF** (`rover/ukf.py`), state `[x, y, heading, speed gain, turn gain]`
  - *Predict*, 20×/s: moves the pose using the wheel commands and the learned gains
    (how many m/s and rad/s the robot really does per unit of command; no encoders needed).
  - *Update*, every scan (~3.3×/s): fuses the scan-matching pose. The noise grows when the
    match is poor, and outliers are rejected with a Mahalanobis gate.
  - The dashboard shows the UKF's accuracy (±cm), current speed, and the learned motor calibration.
- **Mapping** (`rover/slam.py`): occupancy grid + correlative scan matching seeded by the UKF prediction.
  People standing on known floor aren't drawn as walls.
- **Planning** (`rover/planner.py`): A* on the map plus what the lidar sees right now; frontier search for Explore.
- **Navigation** (`rover/navigator.py`): follows the route, re-plans when someone steps in, waits / backs up when blocked.
- **Safety** (`rover/safety.py`): slows within 60 cm and stops within 20 cm of anything in its path, in every mode.
- **Lidar** runs in its own process (`rover/lidar.py`) so it can't stall the rest.

## Tools

| Command | Purpose |
|---|---|
| `./run.sh tools/test_motors.py` | Each motor direction (wheels off the ground) |
| `./run.sh tools/trim_test.py 0.35` | Tune straight-line balance (`[motors] trim`) |
| `./run.sh tools/test_lidar.py` | Lidar rate + nearest object per direction |
| `./run.sh tools/lidar_diag.py` | Scan rate / completeness diagnostics |
| `./run.sh tools/slam_bench.py` | Mapping speed on this Pi |
| `./run.sh tools/drive_until_obstacle.py` | Drive forward, stop before an obstacle |
| `python3 main.py --fake` | Dashboard with fake lidar on a laptop |

## Settings (`config.toml`)

| Section | What |
|---|---|
| `[motors]` | Pins, `trim`, max speed |
| `[lidar]` | Port, `flip`, `angle_offset_deg`, `motor_dtr` |
| `[drive]` | Joystick speeds |
| `[nav]` | Autonomous speed, turning, people handling |
| `[safety]` | Stop / slow distances |
| `[slam]`, `[planner]` | Map size and resolution, clearance from walls |
| `[ukf]` | Filter noise settings and starting motor-gain guesses |
| `[camera]` | Device, resolution, frame rate |
