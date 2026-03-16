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
cd ~/feeding_robot_ws
chmod +x jetson_setup.sh run_feeding.sh
./jetson_setup.sh

# 2. Run (after setup)
source ~/.bashrc

# Simulation mode (Gazebo)
./run_feeding.sh sim

# Real hardware mode (servos + camera + force sensor connected)
./run_feeding.sh real

# Debug mode (each node in its own terminal)
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
| `sim` | `./run_feeding.sh sim` | Launches Gazebo + all feeding nodes |
| `real` | `./run_feeding.sh real` | Launches feeding nodes only (hardware must be running) |
| `predict` | `./run_feeding.sh predict` | ARIMA-FFNN node alone (testing) |
| `fsm` | `./run_feeding.sh fsm` | Feeding FSM alone (testing) |
| `nodes` | `./run_feeding.sh nodes` | Each node in a separate terminal (debugging) |

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
