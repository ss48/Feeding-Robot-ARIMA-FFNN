# Feeding Robot ARIMA-FFNN — Jetson Orin Deployment Notes

## Repository

```
git@github.com:ss48/Feeding-Robot-ARIMA-FFNN.git
```

## Deployment Steps on Jetson Orin

Once pushed, SSH into your Jetson Orin and run:

```bash
# 1. Clone and set up (one-time)
git clone git@github.com:ss48/Feeding-Robot-ARIMA-FFNN.git ~/feeding_robot_ws
##or
git clone https://github.com/ss48/Feeding-Robot-ARIMA-FFNN.git ~/feeding_robot_ws

cd ~/feeding_robot_ws
chmod +x jetson_setup.sh run_feeding.sh
./jetson_setup.sh

# 2. Run (after setup)
source ~/.bashrc

# Real hardware mode (servos + camera + force sensor connected)
./run_feeding.sh real

# Debug mode (each node as a background process, logs in /tmp/feedbot_logs/)
./run_feeding.sh nodes
```

## What `jetson_setup.sh` Does

1. Updates system packages
2. Installs ROS 2 Humble (if not present)
3. Installs all ROS 2 package dependencies (cv_bridge, control_msgs, ros2_control, etc.)
4. Installs Python deps (numpy, opencv, trimesh)
5. Clones the repo (or pulls latest)
6. Runs `rosdep install` for any missing dependencies
7. Builds the workspace with `colcon build`
8. Adds auto-sourcing to `~/.bashrc`

## `run_feeding.sh` Modes

| Mode | Command | What it does |
|------|---------|-------------|
| `sim` | `./run_feeding.sh sim` | Launches Gazebo + all feeding nodes (dev PC only, see below) |
| `real` | `./run_feeding.sh real` | Launches feeding nodes only (hardware must be running) |
| `predict` | `./run_feeding.sh predict` | ARIMA-FFNN node alone (testing) |
| `fsm` | `./run_feeding.sh fsm` | Feeding FSM alone (testing) |
| `nodes` | `./run_feeding.sh nodes` | Each node as a background process, logs in `/tmp/feedbot_logs/` |

## Running Gazebo Simulation (Dev PC Only)

Gazebo simulation requires a **desktop/laptop with a display** — it cannot run on the headless Jetson.

### Prerequisites

- Ubuntu 22.04 (native or WSL2 with GUI support)
- ROS 2 Humble installed
- New Gazebo (Gz) packages:

```bash
sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control
```

### Steps

```bash
# 1. Build the workspace
cd ~/ros2_feedbot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 2a. Launch Gazebo + robot only
ros2 launch feedbot_description gazebo.launch.py

# 2b. Or launch full system (Gazebo + all feeding nodes)
./run_feeding.sh sim
```

This spawns the robot in Gazebo with joint controllers (joint1-4), and in `sim` mode also starts the feeding pipeline nodes after a 10s delay.

## ARIMA-FFNN Node Architecture

### Files Added

| File | Paper Section | Purpose |
|------|---------------|---------|
| `arima_ffnn_node.py` | Alg 1, Eq 1-4, Sec 2.3-2.4 | Hybrid ARIMA+FFNN predictor |
| `fuzzy_controller_node.py` | Sec 2.5.1 | Fuzzy force/angle control per food type |
| `feeding_fsm_node.py` | Fig 1, Sec 2.2 | Full feeding sequence state machine |
| `fusion_node.py` (upgraded) | Sec 2.1 | Multi-sensor fusion + distance estimation |
| `feeding_system.launch.py` | — | Launches the full pipeline |

### Node Data Flow

```
Camera → [vision_node] → /food_visible, /food_center
                              ↓
Force  → [force_node]  → /spoon_force
                              ↓
         [fusion_node]  → /food_error_x, /plate_distance, /mouth_distance
                              ↓
         [arima_ffnn]   → /predicted_state, /prediction_error, /mouth_ready_prediction
                              ↓
         [fuzzy_ctrl]   → /target_force, /target_angle, /feeding_safe
                              ↓
         [feeding_fsm]  → /joint{1-4}_controller/reference, /feeding_state
```

### How the ARIMA-FFNN Maps to the Paper

**`arima_ffnn_node.py`** implements the full pipeline from Equations 1-4:

1. **Time-series decomposition** (Eq. 1): `decompose_series()` splits joint/force history into Trend + Seasonality + Residual using centred moving average
2. **ARIMA on residuals** (Eq. 2): `SimpleARIMA` class fits AR coefficients via OLS on differenced residuals, with MA error correction
3. **FFNN refinement** (Eq. 3): `SimpleFFNN` class — 2-hidden-layer network (tanh activation) trained online via backprop on residual windows
4. **Combined prediction** (Eq. 4): `Y_{t+1} = T[-1] + S[-1] + ARIMA(R) + FFNN(R)`

## Calibration Notes

- **POSES** in `feeding_fsm_node.py` — joint angles for each feeding phase need calibrating to your specific robot/table setup
- **AREA_TO_DISTANCE_K** in `fusion_node.py` — empirical constant mapping detected food area to plate distance
- The ARIMA-FFNN adapts online over iterations (Section 2.5), so performance improves with repeated feeding cycles

## Prerequisites for Jetson Orin

- JetPack 6.x installed (Ubuntu 22.04 based)
- Internet connection for initial setup
- SSH key configured for GitHub access (`ssh-keygen` then add to GitHub)
- For real hardware: Dynamixel servos powered, camera at `/dev/video0`, HX711 force sensor via Arduino

The vision node is running now — the numpy/opencv fix worked.

The ARIMA-FFNN still shows zeros, which is expected because no hardware is connected — there's no camera feed, no force sensor, and no joint states being published, so every node is receiving zero inputs.

To get real data flowing, you need either:

Real hardware (./run_feeding.sh real) — connect the camera, Dynamixel servos, and force sensor
Gazebo simulation (./run_feeding.sh sim) — on a dev PC with a display and ros-humble-ros-gz installed
The system is working correctly — it's just waiting for sensor data. Is the hardware connected to the Jetson, or are you trying to test without it first?


sudo apt install -y ros-humble-ign-ros2-control


sudo apt install -y ros-humble-gz-ros2-control

dpkg -L ros-humble-gz-ros2-control | grep "\.so"

python ros2_feedbot_ws/scripts/perception_demo.py