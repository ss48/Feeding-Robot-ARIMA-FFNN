# Feeding Robot ARIMA-FFNN — Deployment Notes

## Repository

```
git@github.com:ss48/Feeding-Robot-ARIMA-FFNN.git
```

## Deployment Steps on Jetson Orin

Once pushed, SSH into your Jetson Orin and run:

```bash
# 1. Clone and set up (one-time)
git clone git@github.com:ss48/Feeding-Robot-ARIMA-FFNN.git ~/feeding_robot_ws
# or
git clone https://github.com/ss48/Feeding-Robot-ARIMA-FFNN.git ~/feeding_robot_ws

cd ~/feeding_robot_ws
chmod +x jetson_setup.sh run_feeding.sh

# 2. CRITICAL: Copy feeding_robot package into the colcon workspace
#    The feeding_robot package lives at the repo root but colcon only
#    scans ros2_feedbot_ws/src/ — so it must be copied there.
cp -r feeding_robot ros2_feedbot_ws/src/feeding_robot

# 3. Run setup
./jetson_setup.sh

# 4. Run (after setup)
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

### Spacebar Start Control

In `sim` and `real` modes, the system launches all nodes and then **waits for the
user to press SPACE** before starting each feeding cycle. The FSM begins in the
`WAITING` state and only transitions to `IDLE` when it receives a `/feeding_start`
signal (triggered by pressing the spacebar in the terminal).


  Press SPACE → WAITING → IDLE → DETECT_FOOD → LOCATE_FOOD → COLLECT_FOOD →
  DETECT_PATIENT → PRE_FEED → FEED → RETRACT → WAITING (press SPACE again)
```

- **SPACE** — start a feeding cycle
- **Q** — quit and shut down all nodes

You can also trigger the start programmatically:

```bash
ros2 topic pub /feeding_start std_msgs/msg/Bool "{data: true}" --once
```

---

## Running Gazebo Fortress Simulation (Dev PC)

Gazebo simulation requires a **desktop/laptop with a display** — it cannot run on the headless Jetson.

### Prerequisites

- Ubuntu 22.04 (native or WSL2 with GUI support)
- ROS 2 Humble installed
- Gazebo Fortress packages:

```bash
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-image \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-controller-manager
```

### CRITICAL: Package Layout Fix

The `feeding_robot` package **must** be inside the colcon workspace `src/` directory.
The repo ships it at the root, but **colcon only scans `ros2_feedbot_ws/src/`** for
packages. If it's missing there, nothing will work — no URDF, no launch files, no
robot in RViz.

```bash
# Check if it exists in the workspace:
ls ~/feeding_robot_ws/ros2_feedbot_ws/src/feeding_robot/package.xml

# If that file does NOT exist, copy the package in:
cp -r ~/feeding_robot_ws/feeding_robot ~/feeding_robot_ws/ros2_feedbot_ws/src/feeding_robot
```

### Step-by-Step: Building and Loading the Robot

```bash
# ─────────────────────────────────────────────────────
# STEP 1: Ensure feeding_robot is in the workspace
# ─────────────────────────────────────────────────────
cd ~/feeding_robot_ws

# Copy feeding_robot into workspace src/ if not already there
if [ ! -f ros2_feedbot_ws/src/feeding_robot/package.xml ]; then
  echo "Copying feeding_robot into workspace..."
  cp -r feeding_robot ros2_feedbot_ws/src/feeding_robot
fi

# ─────────────────────────────────────────────────────
# STEP 2: Clean build (do this after any structural changes)
# ─────────────────────────────────────────────────────
cd ~/feeding_robot_ws/ros2_feedbot_ws
source /opt/ros/humble/setup.bash

# Remove stale build artifacts
rm -rf build/ install/ log/

# Build ALL packages (feeding_robot + feedbot_fusion + others)
colcon build --symlink-install
source install/setup.bash

# ─────────────────────────────────────────────────────
# STEP 3: Verify both packages are found
# ─────────────────────────────────────────────────────
ros2 pkg prefix feeding_robot
# Should print: ~/feeding_robot_ws/ros2_feedbot_ws/install/feeding_robot
# If it says "Package not found", go back to Step 1

ros2 pkg prefix feedbot_fusion
# Should print: ~/feeding_robot_ws/ros2_feedbot_ws/install/feedbot_fusion

# ─────────────────────────────────────────────────────
# STEP 4 — Terminal 1: Launch Gazebo + robot + RViz
# ─────────────────────────────────────────────────────
ros2 launch feeding_robot gazebo.launch.py

# Wait for ALL of these before opening Terminal 2:
#   [OK] Gazebo window opens (table, plate, 6 coloured fruits, patient head)
#   [OK] RViz opens (robot model appears within ~5 seconds)
#   [OK] Console prints: "Loaded joint_state_broadcaster"
#   [OK] Console prints: "Loaded arm_controller"

# ─────────────────────────────────────────────────────
# STEP 5 — Terminal 2: Launch the feeding pipeline
# ─────────────────────────────────────────────────────
cd ~/feeding_robot_ws/ros2_feedbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch feedbot_fusion feeding_system.launch.py

# This starts 8 nodes:
#   vision_node        — multi-fruit HSV detection from Gazebo camera
#   force_node         — spoon force sensing
#   fusion_node        — EKF sensor fusion (camera + sonar Kalman filter)
#   arima_ffnn         — hybrid ARIMA+FFNN predictor
#   fuzzy_controller   — fuzzy force/angle regulation
#   feeding_fsm        — state machine (SPACE→WAITING→IDLE→DETECT→LOCATE→COLLECT→FEED→RETRACT) + IK forking
#   sonar_bridge       — converts ultrasonic scan → plate/mouth distance
#   mouth_animator     — cycles patient jaw open/close (4s period)
```

### Verifying the Robot Loaded Correctly

```bash
# Check TF tree is publishing (should show world -> base_link -> ... -> feeder_link)
ros2 topic echo /tf --once

# Check joint states are being broadcast
ros2 topic echo /joint_states --once

# Check robot description is available
ros2 topic echo /robot_description --once | head -5

# Check Gazebo sensor topics are bridged
ros2 topic list | grep feeding_robot

# Expected topics:
#   /feeding_robot/camera/image_raw
#   /feeding_robot/ultrasonic/scan

# Check feeding system is running
ros2 topic echo /feeding_state --once

# Manually command the arm (test)
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['base_y_joint','lower_z_joint','upper_z_joint','feeder_joint'], \
    points: [{positions: [0.5, -0.3, 0.4, 0.0], time_from_start: {sec: 2}}]}"
```

### Commanding the Arm (feedbot_description / PID Controllers)

When using `feedbot_description gazebo.launch.py` (i.e. `./run_feeding.sh sim`), the arm
uses individual PID effort controllers per joint. The reference topic expects
`control_msgs/msg/MultiDOFCommand`:

```bash
# Check controllers are active
ros2 control list_controllers

# Move individual joints (feedbot_description PID controllers)
ros2 topic pub /joint1_controller/reference control_msgs/msg/MultiDOFCommand \
  "{dof_names: ['joint1'], values: [1.0]}" --once

ros2 topic pub /joint2_controller/reference control_msgs/msg/MultiDOFCommand \
  "{dof_names: ['joint2'], values: [0.5]}" --once

ros2 topic pub /joint3_controller/reference control_msgs/msg/MultiDOFCommand \
  "{dof_names: ['joint3'], values: [-0.5]}" --once

ros2 topic pub /joint4_controller/reference control_msgs/msg/MultiDOFCommand \
  "{dof_names: ['joint4'], values: [0.3]}" --once
```

**Joint limits (feedbot_description URDF):**

| Joint | Axis | Min (rad) | Max (rad) | Function |
|-------|------|-----------|-----------|----------|
| joint1 | Z | -3.05 | 3.05 | Base rotation |
| joint2 | Y | -1.57 | 1.57 | Shoulder up/down |
| joint3 | Y | -2.09 | 1.31 | Elbow up/down |
| joint4 | Y | -1.57 | 2.01 | Feeder head tilt |

### Monitoring Sensor Topics

```bash
# Camera image (view in rqt or RViz)
ros2 topic hz /feeding_robot/camera/image_raw

# Ultrasonic sonar scan
ros2 topic echo /feeding_robot/ultrasonic/scan --once

# Force/torque on spoon joint
ros2 topic echo /spoon/wrench --once

# Processed sensor outputs (from feeding_system nodes)
ros2 topic echo /spoon_force --once           # scalar force magnitude
ros2 topic echo /sonar_plate_distance --once   # distance to plate (cm)
ros2 topic echo /sonar_mouth_distance --once   # distance to mouth (cm)
ros2 topic echo /food_visible --once           # fruit detected?
ros2 topic echo /food_center --once            # fruit center + area
ros2 topic echo /food_type --once              # detected fruit name
ros2 topic echo /food_bearing --once           # camera bearing (h,v rad) + mono depth (m)

# Joint states
ros2 topic echo /joint_states --once

# 6-state EKF fusion pipeline (verify data flows end-to-end)
ros2 topic echo /food_position_3d --once    # fused 3D food position (metres)
ros2 topic echo /plate_distance --once      # EKF-fused plate distance (cm)
ros2 topic echo /mouth_distance --once      # EKF-fused mouth distance (cm)
ros2 topic echo /food_error_x --once        # visual servoing error
ros2 topic echo /fusion_confidence --once   # overall confidence [0-1]
ros2 topic echo /sensor_health --once       # [vision, force, joints, arima, sonar, bearing]

# Fuzzy controller outputs (driven by EKF fusion)
ros2 topic echo /target_force --once        # desired force (N)
ros2 topic echo /target_angle --once        # desired angle (deg)
ros2 topic echo /feeding_safe --once        # safety signal to FSM

# FSM state (uses all of the above)
ros2 topic echo /feeding_state --once       # current state: IDLE, DETECT_FOOD, etc.
```

### Sensor Fusion Pipeline (6-State EKF + Kalman Filter)

The system uses a **6-state Extended Kalman Filter** (`fusion_node`) that fuses camera
bearing angles with ultrasonic range for precise 3D food localisation:

```
Raw Sensors              Processing Nodes            6-State EKF Fusion        Downstream
─────────────           ──────────────────          ────────────────────       ──────────
Camera (Gz) ──────────→ vision_node ──────→ food_visible ──┐
                                            food_center ───┤
                                            food_bearing ──┤  (bearing_h, bearing_v, mono_depth)
                                                           │
                                                           ├──→ fusion_node ──→ food_position_3d ──→ FSM (IK pickup)
Ultrasonic (Gz) ──────→ sonar_bridge ────→ sonar_plate  ───┤    (6-state EKF)   plate_distance ──→ fuzzy_controller
                                           sonar_mouth  ───┤                    mouth_distance      target_force
                                                           │                    food_error_x        target_angle
Force/Torque (Gz) ────→ force_node ──────→ spoon_force ────┤                    fusion_confidence   feeding_safe
                                                           │                    sensor_health
Joint States ─────────────────────────────────────────────→┤
                                                           │
ARIMA-FFNN ←── joint_states + spoon_force + food_center   ─┘
           └──→ predicted_state, prediction_error ─────────┘
```

**EKF state vector (6D):** `[food_x, food_y, food_z, plate_distance, mouth_distance, force]`

- `food_x` — lateral offset of food from camera centre (metres)
- `food_y` — vertical offset of food from camera centre (metres)
- `food_z` — depth/distance to food along camera axis (metres)
- `plate_distance` — fused distance to plate surface (cm)
- `mouth_distance` — fused distance to patient mouth (cm)
- `force` — contact force on spoon (N)

**Camera + Ultrasonic 3D Food Localisation:**

1. Camera detects food via HSV colour segmentation (vision_node)
2. Pinhole camera model (h_fov=1.047 rad, 640x480) computes bearing angles to food
3. Monocular depth estimated from known fruit diameters vs apparent pixel size
4. Ultrasonic sensor measures range to food on plate (more reliable depth)
5. EKF fuses camera bearing (direction) + ultrasonic range (depth) into stable 3D position
6. FSM uses analytical inverse kinematics on the fused 3D position to compute precise fork joint angles

**3-pass EKF update cycle (per 10 Hz tick):**

1. **Update 1:** Camera bearing + force + joint kinematics (food_x, food_y, food_z, plate_dist, mouth_dist, force)
2. **Update 2:** Ultrasonic range as independent depth measurement (food_z, plate_dist, mouth_dist) — higher confidence than monocular depth
3. **Update 3:** Refine food_x/y using the now-fused depth from updates 1+2

**Fusion features:**
- Confidence-weighted measurement noise (less confident sensors are down-weighted)
- Mahalanobis outlier gating (rejects bad readings)
- Sensor health monitoring with freshness + consistency tracking for 6 sensors (vision, force, joints, arima, sonar, bearing)
- ARIMA-FFNN predicted joints used as process model hint
- Triple update: camera/force/joints → sonar depth → refined bearing projection
- Temporal smoothing via exponential moving average

**Monitor the 3D food localisation:**

```bash
# Fused 3D food position (metres, camera frame)
ros2 topic echo /food_position_3d

# Camera bearing angles + monocular depth
ros2 topic echo /food_bearing

# Ultrasonic raw range (cm)
ros2 topic echo /sonar_plate_distance

# Overall fusion confidence [0-1]
ros2 topic echo /fusion_confidence

# Per-sensor health [vision, force, joints, arima, sonar, bearing]
ros2 topic echo /sensor_health
```

### Troubleshooting: Robot Not Visible in RViz

| Symptom | Cause | Fix |
|---------|-------|-----|
| `Package 'feeding_robot' not found` | Package is at repo root, not in workspace `src/` | Run `cp -r ~/feeding_robot_ws/feeding_robot ~/feeding_robot_ws/ros2_feedbot_ws/src/feeding_robot` then rebuild |
| No robot in RViz, no TF errors | `robot_state_publisher` not running or `robot_description` empty | Check `ros2 topic echo /robot_description --once` returns URDF XML |
| Robot flickers or no TF | `use_sim_time: true` but `/clock` not bridged | Verify `ros2 topic hz /clock` shows ~1000 Hz |
| Robot visible but joints frozen | `joint_state_broadcaster` not active | Run `ros2 control list_controllers` — should show `active` |
| "Controller not loaded" error | `gz_ros2_control` plugin failed | Check Gazebo terminal for errors; ensure `ros-humble-gz-ros2-control` installed |
| Sensors not publishing | Old Classic Gazebo plugins in URDF | Ensure `gazebo_plugins.xacro` has NO `libgazebo_ros_*.so` references (Fortress uses `gz-sim-sensors-system` world plugin instead) |
| RViz shows only grid | Fixed frame wrong | Set RViz Fixed Frame to `world` (not `map` or `base_link`) |

### Key Configuration Files

| File | Purpose |
|------|---------|
| `feeding_robot/description/feeding_robot.urdf.xacro` | Top-level robot description |
| `feeding_robot/description/feeding_robot_core.xacro` | 4-DOF arm links, joints, camera, ultrasonic |
| `feeding_robot/description/gazebo_plugins.xacro` | Fortress sensor definitions + ros2_control plugin |
| `feeding_robot/config/feeding_robot_controllers.yaml` | JointTrajectoryController for `arm_controller` |
| `feeding_robot/worlds/feeding_table.sdf` | Gazebo world: table, plate, 6 fruits, patient head |
| `feeding_robot/config/rviz_config.rviz` | RViz display config (Fixed Frame: `world`) |

---

## Gazebo World Scene

The `feeding_table.sdf` world contains:

- **Table** — 0.8 x 0.6m brown table at 0.74m height
- **Plate** — white cylinder (r=0.12m) on the table
- **6 Coloured Fruits** on the plate:
  - Apple (red sphere, r=0.030m)
  - Strawberry (dark red sphere, r=0.015m)
  - Banana (yellow cylinder, tilted)
  - Grape (purple cluster of 3 small spheres)
  - Orange (orange sphere, r=0.032m)
  - Kiwi (green sphere, r=0.022m)
- **Patient Head** — skin-coloured ellipsoid with eyes, nose, lips, green mouth corner markers
  - Animated jaw joint (`/patient_head/jaw_cmd`) controlled by `mouth_animator_node`
  - Jaw cycles open/closed on a 4-second period

## Robot Sensors (in URDF)

| Sensor | Type | Gazebo Topic | ROS Topic |
|--------|------|-------------|-----------|
| Raspberry Pi Camera V2.1 | camera (640x480, 30Hz) | `feeding_robot/camera/image_raw` | `/feeding_robot/camera/image_raw` |
| HC-SR04 Ultrasonic | gpu_lidar (5 samples, 20Hz) | `feeding_robot/ultrasonic/scan` | `/feeding_robot/ultrasonic/scan` |
| Spoon Force/Torque | force_torque (50Hz, on joint4) | `spoon/wrench` | `/spoon/wrench` |

---

## Node Data Flow (Updated)

```
Camera  → [vision_node]      → /food_visible, /food_center, /food_type, /detected_fruits
                                /food_bearing (bearing_h, bearing_v, mono_depth)
                                    ↓
Sonar   → [sonar_bridge]     → /sonar_plate_distance, /sonar_mouth_distance
                                    ↓
Force   → [force_node]       → /spoon_force
                                    ↓
           [fusion_node]      → /food_position_3d (fused 3D food position, metres)
           (6-state EKF:        /food_error_x, /plate_distance, /mouth_distance
            camera bearing       /fusion_confidence, /sensor_health
            + ultrasonic range
            Kalman filter)
                                    ↓
           [arima_ffnn]       → /predicted_state, /prediction_error
                                    ↓
           [mouth_animator]   → /mouth_ready_prediction, /mouth_open
                                    ↓
           [fuzzy_ctrl]       → /target_force, /target_angle, /feeding_safe
                                    ↓
           [feeding_fsm]      → /arm_controller/joint_trajectory, /feeding_state
           (IK from fused 3D     WAITING → (SPACE) → IDLE → DETECT_FOOD → LOCATE_FOOD →
            food position)        COLLECT_FOOD → DETECT_PATIENT → PRE_FEED → FEED →
                                  RETRACT → WAITING (press SPACE for next cycle)
```

### Joint Names (URDF)

| Joint | Axis | Function |
|-------|------|----------|
| `base_y_joint` | Z (yaw) | Base rotation |
| `lower_z_joint` | Y (pitch) | Shoulder pitch |
| `upper_z_joint` | Y (pitch) | Elbow pitch |
| `feeder_joint` | Y (pitch) | Wrist pitch |

---

## ARIMA-FFNN Node Architecture

### Files

| File | Paper Section | Purpose |
|------|---------------|---------|
| `arima_ffnn_node.py` | Alg 1, Eq 1-4, Sec 2.3-2.4 | Hybrid ARIMA+FFNN predictor |
| `fuzzy_controller_node.py` | Sec 2.5.1 | Fuzzy force/angle control per food type |
| `feeding_fsm_node.py` | Fig 1, Sec 2.2 | FSM with LOCATE_FOOD state + IK-based forking from fused 3D position |
| `fusion_node.py` | Sec 2.1 | 6-state EKF: camera bearing + ultrasonic range → 3D food localisation |
| `vision_node.py` | — | ML food detection (jetson-inference detectNet) with HSV fallback + 3D bearing |
| `sonar_bridge_node.py` | — | Converts ultrasonic LaserScan to distance (cm) |
| `mouth_animator_node.py` | — | Animates patient jaw in Gazebo, publishes mouth state |
| `feeding_system.launch.py` | — | Launches all 8 pipeline nodes |

### How the ARIMA-FFNN Maps to the Paper

**`arima_ffnn_node.py`** implements the full pipeline from Equations 1-4:

1. **Time-series decomposition** (Eq. 1): `decompose_series()` splits joint/force history into Trend + Seasonality + Residual using centred moving average
2. **ARIMA on residuals** (Eq. 2): `SimpleARIMA` class fits AR coefficients via OLS on differenced residuals, with MA error correction
3. **FFNN refinement** (Eq. 3): `SimpleFFNN` class — 2-hidden-layer network (tanh activation) trained online via backprop on residual windows
4. **Combined prediction** (Eq. 4): `Y_{t+1} = T[-1] + S[-1] + ARIMA(R) + FFNN(R)`

## Sensor Fusion (6-State Kalman Filter with Camera + Ultrasonic 3D Localisation)

The `fusion_node.py` implements a **6-state Extended Kalman Filter** for 3D food localisation:

- **State (6D):** `[food_x, food_y, food_z, plate_distance, mouth_distance, force]`
- **Camera bearing** (from `vision_node /food_bearing`) provides direction to food via pinhole camera model
- **Monocular depth** estimated from known fruit diameters (apple=8cm, strawberry=3.5cm, etc.)
- **Ultrasonic range** provides reliable depth measurement — fused with monocular estimate
- **Camera + Ultrasonic fusion:** bearing (direction) + range (depth) → precise 3D food position
- **Triple EKF update:** camera/force/joints → ultrasonic depth → refined bearing projection
- **3D food position** published on `/food_position_3d` for IK-based forking
- **Sensor health monitoring** tracks freshness and consistency of 6 sensors (vision, force, joints, ARIMA, sonar, bearing)

**FSM food localisation flow:**

1. Camera detects food → DETECT_FOOD state
2. **LOCATE_FOOD** state: EKF fuses camera bearing + ultrasonic range until 3D position stabilises (within 2cm over 5 frames)
3. Analytical **inverse kinematics** converts fused (food_x, food_y, food_z) → joint angles [j1, j2, j3, j4]
4. COLLECT_FOOD state: arm moves to IK-computed pickup pose (falls back to predefined pose if IK fails)

---

## Calibration Procedures

### 1. Calibrate Joint Poses (feeding_fsm_node.py)

The FSM uses predefined joint angles for each feeding phase. These must match your
physical table/plate/patient layout. The robot is at the world origin (0,0,0), the
plate is at (0.35, 0, 0.77) and the patient head is at (0.55, 0, 0.85).

**Joint order:** `[base_y_joint, lower_z_joint, upper_z_joint, feeder_joint]`

**Step-by-step in Gazebo:**

```bash
# 1. Launch Gazebo with the robot
ros2 launch feeding_robot gazebo.launch.py

# 2. Use rqt_joint_trajectory_controller to move joints interactively
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller

# Or command joints directly — move one joint at a time:
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['base_y_joint','lower_z_joint','upper_z_joint','feeder_joint'], \
    points: [{positions: [0.0, -0.3, 0.4, 0.0], time_from_start: {sec: 3}}]}"

# 3. Read back the exact joint positions when the spoon is where you want it
ros2 topic echo /joint_states --once
```

**Poses to calibrate (one at a time):**

| Pose | Goal | How to find it |
|------|------|---------------|
| `home` | Arm tucked away, not blocking camera | Start at `[0,0,0,0]`, adjust if arm collides with table |
| `plate_above` | Spoon tip ~5cm above plate center (0.35, 0, 0.82) | Tilt `lower_z_joint` negative (lean forward), `upper_z_joint` positive (extend) |
| `plate_pickup` | Spoon tip touching food on plate (0.35, 0, 0.79) | Lower from `plate_above` — increase `lower_z_joint` magnitude, tilt `feeder_joint` to scoop |
| `pre_feed` | Spoon near patient face (~0.50, 0, 0.85) | Rotate `base_y_joint` towards patient, extend arm |
| `feed` | Spoon at patient mouth (0.55, 0, 0.85) | Extend further from `pre_feed`, tilt `feeder_joint` to aim at mouth |
| `retract` | Pull back halfway between plate and patient | Reduce `base_y_joint`, tuck arm partially |

**Once you find good values, update the POSES dict:**

```python
# feeding_fsm_node.py line 47
POSES = {
    'home':         [0.0,   0.0,   0.0,   0.0],     # your calibrated values
    'plate_above':  [0.0,  -0.3,   0.4,   0.0],
    'plate_pickup': [0.0,  -0.5,   0.6,  -0.3],
    'pre_feed':     [1.2,   0.0,  -0.2,   0.5],
    'feed':         [1.2,   0.3,  -0.4,   0.8],
    'retract':      [0.6,   0.0,   0.0,   0.2],
}
```

### 2. Calibrate AREA_TO_DISTANCE_K (fusion_node.py)

This constant converts the detected food pixel area to a distance in cm using
`distance_cm = K / sqrt(area_pixels)`. The default is `5000.0`.

**How to calibrate:**

```bash
# 1. Launch Gazebo + feeding system
# 2. Echo the detected food area
ros2 topic echo /food_center
# Look at the z field — that is the area in pixels

# 3. Measure the actual distance from camera to plate in Gazebo
#    Camera is on the feeder_link at the arm tip
#    Plate is at z=0.77m. If the arm tip is at z=0.82m looking down,
#    the distance is roughly 5cm = 5.0

# 4. Compute K:
#    K = actual_distance_cm * sqrt(measured_area)
#    Example: distance=25cm, area=400px → K = 25 * 20 = 500
#    Example: distance=25cm, area=40000px → K = 25 * 200 = 5000
```

Update in `fusion_node.py` line 46:

```python
AREA_TO_DISTANCE_K = 5000.0  # adjust based on your camera + distance
```

### 3. Calibrate Fruit HSV Ranges (vision_node.py)

The colour detection thresholds must match your camera's white balance and Gazebo
lighting. The Gazebo sun light and material colours produce specific HSV values.

**How to calibrate:**

```bash
# 1. Save a camera frame from Gazebo
ros2 run image_view image_saver --ros-args -r image:=/feeding_robot/camera/image_raw

# 2. Open in Python and check HSV values of each fruit
python3 -c "
import cv2, numpy as np
img = cv2.imread('left0000.jpg')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Click-based: show HSV at mouse position
def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        h, s, v = hsv[y, x]
        cv2.setWindowTitle('hsv', f'H={h} S={s} V={v} @ ({x},{y})')

cv2.imshow('hsv', img)
cv2.setMouseCallback('hsv', on_mouse)
cv2.waitKey(0)
"
# 3. Note the H, S, V values for each fruit colour
# 4. Set ranges with +-10 margin around the measured values
```

**Current ranges in `vision_node.py`:**

| Fruit | H range | S range | V range |
|-------|---------|---------|---------|
| Apple | 0-10 + 170-180 | 120-255 | 70-255 |
| Strawberry | 0-8 + 172-180 | 150-255 | 100-255 |
| Banana | 20-35 | 100-255 | 100-255 |
| Grape | 125-155 | 50-255 | 50-255 |
| Orange | 10-22 | 150-255 | 150-255 |
| Kiwi | 35-85 | 50-255 | 50-200 |

### 4. Calibrate EKF Noise Parameters (fusion_node.py)

The 6-state Kalman filter performance depends on correct noise covariances.

```python
# Process noise Q — how much state changes per timestep
# Increase if the arm moves fast; decrease for smoother estimates
self.Q = np.diag([0.001, 0.001, 0.005, 1.0, 0.5, 0.05])
#                  ^fx    ^fy    ^fz    ^plate ^mouth ^force
#                  food position (m)    distances (cm)  (N)

# Measurement noise R_base — how noisy each sensor is
# Lower = trust sensor more; Higher = trust prediction more
self.R_base = np.diag([0.01, 0.01, 0.05, 25.0, 10.0, 0.1])
#                       ^fx   ^fy   ^fz   ^plate ^mouth ^force
```

**How to tune:**

```bash
# 1. Monitor 3D food position fusion
ros2 topic echo /food_position_3d      # fused 3D position (metres)
ros2 topic echo /food_bearing          # raw camera bearing + mono depth
ros2 topic echo /sonar_plate_distance  # raw ultrasonic range (cm)

# 2. Monitor distance fusion
ros2 topic echo /plate_distance    # fused estimate (cm)
ros2 topic echo /food_center       # raw camera (z=area)

# 3. If fused estimate is too sluggish (slow to track changes):
#    → Increase Q values (trust process model less)
#    → Decrease R_base values (trust measurements more)

# 4. If fused estimate is too jittery (tracks noise):
#    → Decrease Q values
#    → Increase R_base values

# 5. Monitor sensor health scores
ros2 topic echo /sensor_health
# Array: [vision, force, joints, arima, sonar, bearing] — each 0.0 to 1.0
# If a sensor health is consistently low, check its connection/timeout

# 6. If IK-computed pickup pose is inaccurate:
#    → Check /food_position_3d stability (should settle within 2cm)
#    → Tune food_z noise: lower R_base[2] to trust ultrasonic more
#    → Tune food_x/y noise: lower R_base[0:2] to trust camera bearing more
```

### 5. Calibrate Mouth Animator Timing

```bash
# Change jaw cycle period (seconds) at runtime
ros2 param set /mouth_animator_node period 6.0

# Change how far the jaw opens (radians, negative = open)
ros2 param set /mouth_animator_node max_opening -0.3

# Change the threshold for "mouth is open" (0.0-1.0)
ros2 param set /mouth_animator_node open_threshold 0.4
```

### Quick Calibration Checklist

1. Launch Gazebo: `ros2 launch feeding_robot gazebo.launch.py`
2. Launch feeding system: `ros2 launch feedbot_fusion feeding_system.launch.py`
3. Verify camera sees fruits: `ros2 topic echo /food_visible` should show `true`
4. Verify sonar reads distance: `ros2 topic echo /sonar_plate_distance`
5. Tune POSES using `rqt_joint_trajectory_controller` until arm reaches plate and mouth
6. Tune HSV ranges if fruits are not detected (check `/detected_fruits` topic)
7. Tune EKF if fused distances are unstable (check `/plate_distance`, `/fusion_confidence`)
8. The ARIMA-FFNN self-calibrates online — run several feeding cycles for it to converge

## Prerequisites for Jetson Orin

- JetPack 6.x installed (Ubuntu 22.04 based)
- Internet connection for initial setup
- SSH key configured for GitHub access (`ssh-keygen` then add to GitHub)
- For real hardware: Dynamixel servos powered, camera at `/dev/video0`, HX711 force sensor via Arduino
- CUDA 12.6 toolkit: `sudo apt-get install cuda-nvcc-12-6 cuda-cudart-dev-12-6`

## ML Food Detection (jetson-inference)

The vision node uses **NVIDIA jetson-inference** (`detectNet`) for ML-based object
detection on Jetson hardware, with HSV colour segmentation as fallback for Gazebo.

**Current status:** The launch file defaults to `detection_method: hsv` because
TensorRT 10.3 on JetPack 6.x dropped support for legacy UFF/Caffe model formats.
Change to `'auto'` or `'ml'` in `feeding_system.launch.py` once an ONNX model is configured.

### Installing jetson-inference (Jetson Orin / JetPack 6.x)

```bash
# 1. Install CUDA compiler (if not already present)
sudo apt-get install cuda-nvcc-12-6 cuda-cudart-dev-12-6

# Verify nvcc is available
which nvcc   # should show /usr/local/cuda-12.6/bin/nvcc

# 2. Clone and build jetson-inference (install system-wide, NOT in the ROS workspace)
cd ~
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build && cd build

# Set CUDA path explicitly
export CUDA_HOME=/usr/local/cuda-12.6
export PATH=$CUDA_HOME/bin:$PATH
cmake -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.6 ../

# If cmake fails on SM arch, force Orin's compute capability:
# cmake -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.6 -DCUDA_NVCC_FLAGS="-gencode arch=compute_87,code=sm_87" ../

make -j$(nproc)

# 3. If build fails on npymath (NumPy 2.0+ removed it), create a stub:
sudo ar rcs /usr/lib/aarch64-linux-gnu/libnpymath.a
make -j$(nproc)   # retry

sudo make install
sudo ldconfig

# 4. Install Python bindings manually
#    (the build system doesn't always install them to the right path)
SITE=/usr/local/lib/python3.10/dist-packages

sudo cp ~/jetson-inference/build/aarch64/lib/python/3.10/jetson_utils_python.so $SITE/jetson_utils_python.so
sudo cp ~/jetson-inference/build/aarch64/lib/python/3.10/jetson_inference_python.so $SITE/jetson_inference_python.so

# Copy the Python wrapper packages
sudo cp -r ~/jetson-inference/python/python/jetson_inference $SITE/
sudo cp -r ~/jetson-inference/python/python/jetson $SITE/
sudo cp -r ~/jetson-inference/python/python/Jetson $SITE/
sudo cp -r ~/jetson-inference/utils/python/python/jetson_utils $SITE/ 2>/dev/null
sudo cp -r ~/jetson-inference/utils/python/python/jetson $SITE/ 2>/dev/null

# 5. Verify
python3 -c "import jetson_inference; print('OK')"
python3 -c "import jetson_utils; print('OK')"
```

### Known Issue: TensorRT 10.3 + Legacy Models

JetPack 6.x ships TensorRT 10.3 which **does not support** legacy UFF/Caffe model
formats (`.uff`, `.caffemodel`). The default SSD-Mobilenet-v2 model in jetson-inference
uses `.uff` format and will fail with:

```
[TRT] TensorRT 10.3 does not support legacy caffe models
[TRT] detectNet -- failed to initialize.
```

**Workarounds (in order of preference):**

1. **Use HSV detection** (current default) — works out of the box for Gazebo and
   real food with tuned colour ranges

2. **Download an ONNX model** — The vision node will auto-detect ONNX models at
   `~/jetson-inference/data/networks/ssd_mobilenet_v2.onnx`

3. **Install OpenSSL 1.1** for TAO converter — some TAO/etlt models need this:
   ```bash
   # The tao-converter needs libcrypto.so.1.1
   sudo apt install libssl1.1
   # If not available, download from Ubuntu 20.04 repos
   ```

4. **Use PyTorch + torchvision** — alternative ML approach (requires separate setup)

### Detection Method Configuration

The vision node parameter `detection_method` controls which detector is used:

| Value | Behaviour |
|-------|-----------|
| `hsv` | Always use HSV colour segmentation (default in launch file) |
| `auto` | Use ML if jetson-inference loads successfully, else HSV fallback |
| `ml` | Force ML detection (warns and falls back to HSV if unavailable) |

**Change in launch file** (`feeding_system.launch.py`):

```python
# Currently set to 'hsv' — change to 'auto' when ML model works
parameters=[{'detection_method': 'hsv'}],
```

**Or override at runtime:**

```bash
ros2 run feedbot_fusion vision_node --ros-args -p detection_method:=auto
```

### COCO Food Classes Detected (ML mode)

The SSD-Mobilenet-v2 model detects these food items from the COCO dataset:

| Class ID | Name | Class ID | Name |
|----------|------|----------|------|
| 52 | banana | 53 | apple |
| 54 | sandwich | 55 | orange |
| 56 | broccoli | 57 | carrot |
| 58 | hot_dog | 59 | pizza |
| 60 | donut | 61 | cake |

### Tuning Detection

```bash
# Adjust ML confidence threshold (default: 0.35)
ros2 run feedbot_fusion vision_node --ros-args -p ml_confidence:=0.25

# Change camera topic (e.g. for real hardware)
ros2 run feedbot_fusion vision_node --ros-args -p camera_topic:=/camera/image_raw
```

### Robot Home Pose

The robot starts in the **WAITING** state with the arm tilted forward so the camera
faces the plate. Home pose: `[0.0, 0.5, -0.8, -0.5]` (joint2 forward, joint3 elbow
down, joint4 tilted to look at plate). Press SPACE to start the feeding cycle.

## Standalone Perception Demo (No ROS Required)

```bash
python ros2_feedbot_ws/scripts/perception_demo.py
```

- **With display** (Windows/desktop Linux): Opens a live OpenCV window with interactive keys (`q` quit, `o` outlier storm, `s` sonar failure)
- **Headless** (SSH/Jetson without `$DISPLAY`): Saves a 300-frame video to `perception_demo_output.avi`, prints metrics to console every 30 frames, auto-enables outlier storm at frames 150-210 to demonstrate Kalman filter resilience

---

## Verifying Computer Vision + Gazebo + RViz Integration

This section covers how to verify that jetson-inference (or HSV fallback), Gazebo simulation, and RViz are all working together end-to-end.

### Step-by-Step Launch Sequence

You need **3 terminals** (all sourced). Alternatively, use `./run_feeding.sh sim` which handles terminals 1 and 2 automatically.

```bash
# ─────────────────────────────────────────────────────
# Terminal 1: Launch Gazebo + Robot + Controllers + RViz
# ─────────────────────────────────────────────────────
cd ~/feeding_robot_ws/ros2_feedbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch feeding_robot gazebo.launch.py

# Wait until you see:
#   [OK] Gazebo window opens (table, plate, fruits, patient head)
#   [OK] RViz opens (robot model visible)
#   [OK] "Loaded joint_state_broadcaster" in console
#   [OK] "Loaded arm_controller" in console
# This takes ~12 seconds.

# ─────────────────────────────────────────────────────
# Terminal 2: Launch Feeding System (all 8 nodes)
# ─────────────────────────────────────────────────────
cd ~/feeding_robot_ws/ros2_feedbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch feedbot_fusion feeding_system.launch.py

# This starts: vision_node, force_node, sonar_bridge, fusion_node,
#              arima_ffnn, fuzzy_controller, feeding_fsm, mouth_animator

# ─────────────────────────────────────────────────────
# Terminal 3: Monitoring & Triggering
# ─────────────────────────────────────────────────────
cd ~/feeding_robot_ws/ros2_feedbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Trigger a feeding cycle:
ros2 topic pub /feeding_start std_msgs/msg/Bool "{data: true}" --once
# Or press SPACE in Terminal 2 if using run_feeding.sh
```

### Verifying Each Subsystem

#### 1. Gazebo Simulation

```bash
# Check Gazebo clock is publishing (should be ~1000 Hz)
ros2 topic hz /clock

# Check camera bridge is active (should be ~30 Hz)
ros2 topic hz /feeding_robot/camera/image_raw

# Check ultrasonic bridge is active (should be ~20 Hz)
ros2 topic hz /feeding_robot/ultrasonic/scan

# Check force/torque sensor
ros2 topic echo /spoon/wrench --once

# Check all expected topics exist
ros2 topic list | grep feeding_robot
# Expected:
#   /feeding_robot/camera/image_raw
#   /feeding_robot/ultrasonic/scan
```

#### 2. Computer Vision (jetson-inference / HSV Fallback)

```bash
# Is food being detected?
ros2 topic echo /food_visible
# Should show: data: true (when camera sees food on the plate)

# What food type is detected?
ros2 topic echo /food_type
# Should show: data: "apple" (or banana, orange, etc.)

# Food centre coordinates + pixel area
ros2 topic echo /food_center
# x = pixel x, y = pixel y, z = area in pixels

# Camera bearing angles + monocular depth estimate
ros2 topic echo /food_bearing
# x = bearing_h (rad), y = bearing_v (rad), z = estimated_depth (m)

# Multi-fruit detection array
ros2 topic echo /detected_fruits

# Check detection method being used (on dev PC it will be HSV)
# On Jetson Orin with jetson-inference installed, it uses ML (detectNet SSD-Mobilenet-v2)
# On dev PC or when jetson-inference is unavailable, it falls back to HSV colour segmentation
```

**If `/food_visible` always shows `false`:**

- Verify camera is publishing: `ros2 topic hz /feeding_robot/camera/image_raw`
- Check the arm's home pose points the camera at the plate (the robot starts in home pose `[0.0, 0.5, -0.8, -0.5]` with camera facing the plate)
- If using HSV fallback, the Gazebo fruit colours must match the HSV ranges — see the Calibration section above
- Save a frame and inspect HSV values:
  ```bash
  ros2 run image_view image_saver --ros-args -r image:=/feeding_robot/camera/image_raw
  ```

#### 3. Sensor Fusion (EKF)

```bash
# Fused 3D food position (the main output — metres, camera frame)
ros2 topic echo /food_position_3d

# Per-sensor health scores [vision, force, joints, arima, sonar, bearing]
# Each value 0.0 to 1.0 — low values indicate sensor issues
ros2 topic echo /sensor_health

# Overall fusion confidence [0-1]
ros2 topic echo /fusion_confidence

# EKF-fused plate and mouth distances (cm)
ros2 topic echo /plate_distance
ros2 topic echo /mouth_distance

# Visual servoing error
ros2 topic echo /food_error_x
```

#### 4. RViz Visualisation

```bash
# If RViz didn't launch automatically, start it manually:
rviz2 -d ~/feeding_robot_ws/ros2_feedbot_ws/src/feeding_robot/config/rviz_config.rviz
```

**In RViz, verify:**

- **Fixed Frame** is set to `world` (not `map` or `base_link`)
- **RobotModel** display shows the 4-DOF arm with all links
- **TF** display shows the transform tree: `world → base_link → ... → feeder_link`
- Add an **Image** display (topic: `/feeding_robot/camera/image_raw`) to see live camera feed
- Add a **LaserScan** display (topic: `/feeding_robot/ultrasonic/scan`) to see ultrasonic rays

#### 5. Controllers & Arm Motion

```bash
# Check controllers are loaded and active
ros2 control list_controllers
# Expected:
#   joint_state_broadcaster [active]
#   arm_controller          [active]

# Verify joint states are broadcasting
ros2 topic echo /joint_states --once

# Check TF tree
ros2 topic echo /tf --once

# Check robot description is loaded
ros2 topic echo /robot_description --once | head -5
```

#### 6. FSM State Machine

```bash
# Monitor current FSM state
ros2 topic echo /feeding_state
# States cycle: WAITING → IDLE → DETECT_FOOD → LOCATE_FOOD → COLLECT_FOOD →
#               DETECT_PATIENT → PRE_FEED → FEED → RETRACT → WAITING

# Trigger a feeding cycle
ros2 topic pub /feeding_start std_msgs/msg/Bool "{data: true}" --once

# Watch the arm move in both Gazebo and RViz simultaneously
# The FSM uses IK from the fused 3D food position to compute joint angles
```

#### 7. Fuzzy Controller

```bash
# Check fuzzy controller outputs (driven by EKF fusion)
ros2 topic echo /target_force    # desired force (N) based on food type
ros2 topic echo /target_angle    # desired angle (deg) based on food type
ros2 topic echo /feeding_safe    # safety signal sent to FSM
```

### Quick Diagnostic Checklist

| # | Check | Command | Expected Result |
|---|-------|---------|-----------------|
| 1 | All nodes running | `ros2 node list` | 8+ nodes: `vision_node`, `fusion_node`, `feeding_fsm`, `arima_ffnn`, `fuzzy_controller`, `force_node`, `sonar_bridge`, `mouth_animator` |
| 2 | Camera publishing | `ros2 topic hz /feeding_robot/camera/image_raw` | ~30 Hz |
| 3 | Ultrasonic publishing | `ros2 topic hz /feeding_robot/ultrasonic/scan` | ~20 Hz |
| 4 | Food detected | `ros2 topic echo /food_visible` | `data: true` |
| 5 | Food type identified | `ros2 topic echo /food_type` | `data: "apple"` (or other fruit) |
| 6 | Bearing computed | `ros2 topic echo /food_bearing` | Non-zero x, y, z values |
| 7 | Sonar range | `ros2 topic echo /sonar_plate_distance` | Distance in cm (e.g. 5-30) |
| 8 | 3D position fused | `ros2 topic echo /food_position_3d` | x, y, z in metres |
| 9 | Fusion confidence | `ros2 topic echo /fusion_confidence` | Value between 0.0-1.0 (higher is better) |
| 10 | Sensor health | `ros2 topic echo /sensor_health` | 6 values, each 0.0-1.0 |
| 11 | Controllers active | `ros2 control list_controllers` | `joint_state_broadcaster` + `arm_controller` both `[active]` |
| 12 | FSM responding | `ros2 topic echo /feeding_state` | State name (e.g. `WAITING`, `IDLE`) |
| 13 | Arm moves | Trigger with SPACE or `/feeding_start` | Robot arm moves in both Gazebo and RViz |
| 14 | RViz shows robot | Visual check | Robot model visible, TF frames shown |

### Integration Test: Full Feeding Cycle

To confirm everything is working end-to-end:

1. **Launch Gazebo** (Terminal 1): `Terminal 1: Robot + Gazebo + RViz (the arm + face you want)
ros2 launch feedbot_description gazebo.launch.py
2. **Wait 12s**, confirm Gazebo window shows table with fruits and RViz shows robot
3. **Launch feeding system** (Terminal 2): `ros2 launch feedbot_fusion feeding_system.launch.py`
4. **Monitor** (Terminal 3): `ros2 topic echo /feeding_state`
5. **Trigger** (Terminal 3): `ros2 topic pub /feeding_start std_msgs/msg/Bool "{data: true}" --once`
6. **Observe the full state cycle:**
   - `WAITING` → `IDLE` — system initialises
   - `IDLE` → `DETECT_FOOD` — vision_node detects food via HSV/ML
   - `DETECT_FOOD` → `LOCATE_FOOD` — EKF fuses camera + ultrasonic for 3D position
   - `LOCATE_FOOD` → `COLLECT_FOOD` — IK computes joint angles, arm moves to food
   - `COLLECT_FOOD` → `DETECT_PATIENT` — arm picks up food
   - `DETECT_PATIENT` → `PRE_FEED` → `FEED` — arm moves to patient mouth
   - `FEED` → `RETRACT` → `WAITING` — arm retracts, cycle complete
7. **Verify in Gazebo:** arm physically moves to plate, picks up food, moves to patient
8. **Verify in RViz:** robot model mirrors Gazebo arm motion in real-time
9. **Press SPACE again** to run another cycle

### Common Integration Issues

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Food never detected | Camera not bridged or HSV ranges wrong | Check `ros2 topic hz /feeding_robot/camera/image_raw`, calibrate HSV ranges |
| FSM stuck on `DETECT_FOOD` | `/food_visible` never becomes `true` | Verify vision_node is running, check home pose points camera at plate |
| FSM stuck on `LOCATE_FOOD` | 3D position not stabilising | Check `/fusion_confidence`, ensure both camera and sonar are publishing |
| Arm doesn't move | Controllers not active | Run `ros2 control list_controllers`, check for Gazebo plugin errors |
| RViz and Gazebo out of sync | `use_sim_time` mismatch | Ensure all nodes use `use_sim_time: true`, check `/clock` topic is bridged |
| `jetson_inference` import fails | Not installed or Python path wrong | Use HSV fallback (`detection_method: hsv`) or install per instructions above |
| Sonar reads 0 or NaN | Ultrasonic not bridged or no objects in range | Check `ros2 topic echo /feeding_robot/ultrasonic/scan`, verify plate is within 2-400cm |
| Low fusion confidence | One or more sensors unhealthy | Check `/sensor_health` — identify which sensor has low score and investigate |
| IK pickup misses food | 3D position inaccurate | Tune EKF noise parameters (see Calibration section), check `AREA_TO_DISTANCE_K` |

### Monitoring All Topics at Once

```bash
# List all active topics with their types
ros2 topic list -t

# Monitor multiple topics simultaneously (use separate terminal per topic)
# Or use rqt for a GUI-based overview:
rqt

# In rqt:
#   Plugins → Topics → Topic Monitor (see all topic rates)
#   Plugins → Visualisation → Image View (see camera feed)
#   Plugins → Robot Tools → Joint Trajectory Controller (move joints interactively)
```

### Using `run_feeding.sh` for Quick Integration Test

The simplest way to test everything together:

```bash
cd ~/feeding_robot_ws
./run_feeding.sh sim
# Wait for "System ready!" message
# Press SPACE to start a feeding cycle
# Press Q to quit
# Logs saved to /tmp/feedbot_logs/
```

Check logs if something fails:

```bash
cat /tmp/feedbot_logs/gazebo.log           # Gazebo + controller errors
cat /tmp/feedbot_logs/feeding_system.log   # All 8 feeding node outputs
```

The deployment notes have been updated. I added a comprehensive new section at the end of deploymentNotes.md covering:

Step-by-step launch sequence — 3-terminal setup for Gazebo, feeding system, and monitoring
Subsystem verification — commands to check each component individually:
Gazebo simulation (clock, camera, ultrasonic, force bridges)
Computer vision (food detection, bearing, HSV vs ML mode)
Sensor fusion (EKF 3D position, confidence, health scores)
RViz visualisation (displays to add, fixed frame setting)
Controllers & arm motion
FSM state machine
Fuzzy controller outputs
14-point diagnostic checklist table with commands and expected results
Full feeding cycle integration test — step-by-step walkthrough of triggering and observing the complete state cycle
Common integration issues table with symptoms, causes, and fixes
Monitoring tips using rqt and run_feeding.sh sim for quick testing

---

## Real Hardware Setup (Raspberry Pi 4 8GB) — Updated 2026-04-09

### Platform

- Raspberry Pi 4 (8 GB) running Ubuntu 22.04 + ROS 2 Humble
- Dynamixel XM430-W350R servos via U2D2 (ROBOTIS OpenCR)
- Teensy microcontroller for HX711 force sensor + HC-SR04 ultrasonic
- Raspberry Pi Camera V2.1 at `/dev/video0`

### USB Device Mapping (udev Rules)

USB ports can swap on reboot. Use udev rules for stable device names:

```bash
sudo bash -c 'cat > /etc/udev/rules.d/99-feedbot.rules << "EOF"
SUBSYSTEM=="tty", ATTRS{manufacturer}=="Teensyduino", SYMLINK+="teensy", MODE="0666"
SUBSYSTEM=="tty", ATTRS{manufacturer}=="ROBOTIS", SYMLINK+="dynamixel", MODE="0666"
EOF'
sudo udevadm control --reload-rules
sudo udevadm trigger
```

This creates `/dev/dynamixel` and `/dev/teensy` symlinks that always point to the correct devices.

### Dynamixel Servo Configuration

| Servo ID | Joint | Role |
|----------|-------|------|
| 11 | `base_y_joint` | Base yaw rotation |
| 12 | `lower_z_joint` | Shoulder pitch |
| 13 | `upper_z_joint` | Elbow pitch |
| 14 | `feeder_joint` | Wrist pitch (feeder head tilt) |
| 15 | (unused) | Was gripper — replaced with feeder face + sensors |

- **Baud rate:** 1000000 (changed from factory default 57600)
- **Port:** `/dev/dynamixel` (symlink to `/dev/ttyACMx`)
- **Protocol:** Dynamixel Protocol 2.0

### Teensy Sensor Board

The Teensy reads HX711 force sensor and HC-SR04 ultrasonic sensor, sending data over serial at 115200 baud.

| Sensor | Teensy Pin | Serial Output Format |
|--------|-----------|---------------------|
| HX711 DOUT | Pin 2 | `Load Cell Reading: 9.2 g` |
| HX711 SCK | Pin 3 | |
| HC-SR04 TRIG | Pin 4 | `Distance (cm): 8.86` |
| HC-SR04 ECHO | Pin 5 | |

- **Port:** `/dev/teensy` (symlink to `/dev/ttyACMx`)
- **Baud rate:** 115200
- **Firmware:** `firmware/teensy_sensors.ino`

### Feeder Head (Replaced Gripper)

The standard Open Manipulator-X gripper (servo ID 15) was removed and replaced with a custom feeder face:

| Component | Mesh File | URDF Link |
|-----------|-----------|-----------|
| Feeder face | `frontface.stl` | `feeder_link` |
| Raspberry Pi Camera V2.1 | `camera.stl` | `camera_link` |
| HC-SR04 Ultrasonic Sensor | `sonar.stl` | `ultrasonic_link` |
| HX711 Force Sensor | `forceSensor.stl` | `load_cell_frame` |

All meshes are in CAD coordinates. The URDF applies offsets relative to the feeder joint at CAD position (194, 0, 351) mm.

### One-Time Setup

```bash
# Install ROS 2 dependencies
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-moveit ros-humble-v4l2-camera ros-humble-rqt-image-view

# Install Python dependencies
pip3 install pyserial

# Add user to dialout group (avoids chmod every boot)
sudo usermod -aG dialout $USER
# Then reboot

# Create udev rules (see above)

# Flash Teensy with firmware/teensy_sensors.ino via Arduino IDE
```

### Hardware Launch

```bash
cd ~/feeding_robot_ws/ros2_feedbot_ws
colcon build --symlink-install --packages-select feeding_robot feedbot_fusion
source install/setup.bash
ros2 launch feeding_robot hardware.launch.py
```

This launches all hardware nodes:
- `ros2_control_node` — Dynamixel hardware interface (4 joints, no gripper)
- `robot_state_publisher` — TF tree from URDF
- `joint_state_broadcaster` + `arm_controller` — joint trajectory controller at 100 Hz
- `v4l2_camera_node` — Pi camera → `/feeding_robot/camera/image_raw` (640x480)
- `teensy_bridge` — serial bridge → `/spoon_force`, `/spoon/wrench`, `/feeding_robot/ultrasonic/range`, `/sonar_raw_cm`
- `sonar_bridge` — routes ultrasonic data to `/sonar_plate_distance` or `/sonar_mouth_distance` based on feeding state

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `port_name` | `/dev/dynamixel` | Dynamixel USB port |
| `teensy_port` | `/dev/teensy` | Teensy serial port |
| `camera_device` | `/dev/video0` | Camera device path |
| `use_meshes` | `true` | Use STL mesh files in URDF |

```bash
ros2 launch feeding_robot hardware.launch.py port_name:=/dev/dynamixel teensy_port:=/dev/teensy
```

### Verifying Sensor Data

```bash
source ~/feeding_robot_ws/ros2_feedbot_ws/install/setup.bash

# Joint states
ros2 topic echo /joint_states --once

# Force sensor (Newtons)
ros2 topic echo /spoon_force --once

# Sonar distance (cm)
ros2 topic echo /sonar_raw_cm --once

# Camera frame rate
ros2 topic hz /feeding_robot/camera/image_raw

# View camera (requires display)
ros2 run rqt_image_view rqt_image_view /feeding_robot/camera/image_raw

# List all sensor topics
ros2 topic list | grep -E "force|sonar|spoon|camera|joint"
```

### Moving Joints

```bash
source ~/feeding_robot_ws/ros2_feedbot_ws/install/setup.bash

# Move all joints to home position (0,0,0,0) over 3 seconds
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [base_y_joint, lower_z_joint, upper_z_joint, feeder_joint],
    points: [{
      positions: [0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 3, nanosec: 0}
    }]
  }
}"
```

### Scanning/Diagnosing Dynamixel Servos

```bash
python3 -c "
from dynamixel_sdk import *
port = PortHandler('/dev/dynamixel')
packet = PacketHandler(2.0)
port.openPort()
for baud in [57600, 115200, 1000000]:
    port.setBaudRate(baud)
    print(f'Scanning baud rate: {baud}...')
    for i in range(0, 20):
        model, result, error = packet.ping(port, i)
        if result == 0:
            print(f'  FOUND servo ID: {i}, model: {model}')
print('Scan complete.')
port.closePort()
"
```

### Changing Servo Baud Rate

If servos are at factory 57600, change to 1000000:

```bash
python3 -c "
from dynamixel_sdk import *
port = PortHandler('/dev/dynamixel')
packet = PacketHandler(2.0)
port.openPort()
port.setBaudRate(57600)
for dxl_id in [11, 12, 13, 14]:
    packet.write1ByteTxRx(port, dxl_id, 64, 0)
    result, error = packet.write1ByteTxRx(port, dxl_id, 8, 3)
    print(f'ID {dxl_id}: {\"OK\" if result == 0 else \"FAILED\"} ')
port.closePort()
print('Power cycle the servos now.')
"
```

### Testing Teensy Serial Directly

```bash
python3 -c "
import serial, time
s = serial.Serial('/dev/teensy', 115200, timeout=2)
time.sleep(2)
for i in range(5):
    print(s.readline().decode().strip())
s.close()
"
```

Expected output:
```
Distance (cm): 8.86
Load Cell Reading: 9.2 g
Distance (cm): 8.84
Load Cell Reading: 9.1 g
```

### Key Files

| File | Purpose |
|------|---------|
| `feeding_robot/description/feeding_robot_ros2_control.xacro` | Dynamixel hardware interface (servo IDs, baud rate, GPIO blocks) |
| `feeding_robot/description/feeding_robot_core.xacro` | URDF links/joints, mesh references, sensor mounts |
| `feeding_robot/config/feeding_robot_controllers.yaml` | Controller manager (joint trajectory controller, 100 Hz) |
| `feeding_robot/launch/hardware.launch.py` | Real hardware launch (servos + camera + sensors) |
| `feeding_robot/launch/gazebo.launch.py` | Simulation launch (Gazebo + bridges) |
| `feeding_robot/meshes/visual/` | STL meshes (frontface, camera, sonar, forceSensor, arm links) |
| `feedbot_fusion/teensy_bridge_node.py` | Teensy serial bridge (force + sonar → ROS topics) |
| `feedbot_fusion/sonar_bridge_node.py` | Routes sonar to plate/mouth topics by feeding state |
| `firmware/teensy_sensors.ino` | Teensy firmware (HX711 + HC-SR04) |

### ROS 2 Topic Map (Real Hardware)

| Topic | Type | Source | Rate |
|-------|------|--------|------|
| `/joint_states` | JointState | joint_state_broadcaster | 50 Hz |
| `/feeding_robot/camera/image_raw` | Image | v4l2_camera | 30 Hz |
| `/spoon_force` | Float64 | teensy_bridge | ~10 Hz |
| `/spoon/wrench` | WrenchStamped | teensy_bridge | ~10 Hz |
| `/feeding_robot/ultrasonic/range` | Range | teensy_bridge | ~10 Hz |
| `/sonar_raw_cm` | Float64 | teensy_bridge | ~10 Hz |
| `/sonar_plate_distance` | Float64 | sonar_bridge | ~10 Hz |
| `/sonar_mouth_distance` | Float64 | sonar_bridge | ~10 Hz |

### Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `Failed to open port` | Permission denied or wrong port | Check `ls /dev/dynamixel /dev/teensy`; recreate udev rules |
| `No status packet` (ping fails) | Wrong baud rate or no power | Scan all baud rates; check 12V supply is on |
| `SYNC_READ_FAIL` continuously | Baud too slow for update rate | Ensure servos at 1000000 baud |
| `plugin not found` | Wrong plugin name | Use `dynamixel_hardware_interface/DynamixelHardware` |
| Ports swapped after reboot | USB enumeration order changed | Use udev symlinks (`/dev/dynamixel`, `/dev/teensy`) |
| No force/sonar data | Teensy not flashed or disconnected | Test with direct serial read (see above) |
| Teensy `OSError: I/O error` | Teensy disconnected mid-read | Bridge auto-reconnects; check USB cable |
| Camera not publishing | Wrong device or not enabled | Check `ls /dev/video*`; `sudo raspi-config` → enable camera |
| Meshes missing in RViz | STL files not in workspace | Ensure `frontface.stl`, `camera.stl`, `sonar.stl`, `forceSensor.stl` in `meshes/visual/` |

---

## Computer Vision & AI Intelligence — Updated 2026-04-09

### Overview

The robot operates in a fully autonomous feeding loop without requiring a spacebar press:

```
detect plate → detect food → fork food → find user's face →
wait for mouth open → feed → retract → repeat
```

### Dependencies

```bash
pip3 install mediapipe>=0.10.9
```

MediaPipe Face Mesh runs on Pi 4 CPU at ~5-8 fps (no GPU needed). The `process_every_n` parameter (default 3) skips frames to maintain acceptable throughput.

### Face Detection Node (`face_node.py`)

**File:** `feedbot_fusion/feedbot_fusion/face_node.py`

Uses MediaPipe Face Mesh (478 landmarks with iris refinement) for:

| Feature | Method | Landmarks Used |
|---------|--------|----------------|
| Face detection | MediaPipe Face Mesh | All 478 |
| Mouth open/closed | Mouth Aspect Ratio (MAR) | 13 (upper lip), 14 (lower lip), 78 (left corner), 308 (right corner) |
| Eye alertness | Eye Aspect Ratio (EAR) | Left: 33,160,158,133,153,144; Right: 362,385,387,263,373,380 |
| Face depth | Inter-pupillary distance (IPD=63mm) via pinhole model | 468 (left iris), 473 (right iris) |
| Expression | "ready" (eyes open + face toward camera) or "not_ready" | EAR + face mesh confidence |

**MAR formula:** `|upper_lip(13) - lower_lip(14)| / |left_corner(78) - right_corner(308)|`
- Mouth open when MAR > 0.3 (configurable via `mar_threshold` parameter)

**Published topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `/face_detected` | Bool | Face found in frame |
| `/face_center` | Point | cx, cy (pixels), z=face_area |
| `/face_bearing` | Point | bearing_h, bearing_v (rad), estimated_depth (m) |
| `/mouth_open` | Bool | MAR > threshold |
| `/mouth_aspect_ratio` | Float64 | Raw MAR value for tuning |
| `/face_expression` | String | "ready" / "not_ready" / "no_face" |
| `/mouth_ready_prediction` | Bool | face detected + mouth open + ready |

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_topic` | `/feeding_robot/camera/image_raw` | Camera input |
| `mar_threshold` | 0.3 | Mouth Aspect Ratio threshold for "open" |
| `ear_threshold` | 0.2 | Eye Aspect Ratio threshold for "alert" |
| `detection_confidence` | 0.5 | MediaPipe face detection confidence |
| `tracking_confidence` | 0.5 | MediaPipe face tracking confidence |
| `max_num_faces` | 1 | Only track one user |
| `process_every_n` | 3 | Process every Nth frame (~8-10fps on Pi 4) |

### Plate Detection (in `vision_node.py`)

Added shape-based plate detection via contour circularity analysis:
- Canny edge detection → contour finding → circularity check
- Large, roughly circular contour (area > 5000px, circularity > 0.4) = plate
- Publishes `/plate_detected` (Bool)

### Autonomous FSM (in `feeding_fsm_node.py`)

The state machine now operates autonomously with camera-based face detection:

```
WAITING ──[plate+food detected (auto) OR spacebar (manual)]──> IDLE
IDLE ──[food_visible]──> DETECT_FOOD
DETECT_FOOD ──[food confirmed]──> LOCATE_FOOD
LOCATE_FOOD ──[EKF stable + IK computed]──> COLLECT_FOOD
COLLECT_FOOD ──[force > 2N]──> DETECT_PATIENT
DETECT_PATIENT ──[face_detected + expression=="ready"]──> PRE_FEED
PRE_FEED ──[face-approach IK + at target + safe]──> FEED
FEED ──[mouth_open for 5 consecutive frames]──> RETRACT
RETRACT ──[at target]──> WAITING (auto-restart)
```

**Key changes from previous version:**

| State | Before | After |
|-------|--------|-------|
| WAITING → IDLE | Spacebar required | Auto: plate + food detected (spacebar still works) |
| DETECT_PATIENT | ARIMA heuristic (not real detection) | Camera face detection via face_node |
| PRE_FEED | Fixed pre_feed pose | Dynamic face-approach IK from face bearing |
| FEED | ARIMA mouth prediction | Real mouth-open detection (5 consecutive frames = 0.5s) |
| RETRACT → WAITING | Manual spacebar for next cycle | Auto-restart when plate+food still visible |

### Face-Approach Inverse Kinematics

New method `_compute_face_approach_ik(bearing_h, bearing_v, depth)`:
- Computes joint angles to position fork ~5cm in front of user's face
- Uses same 2-link analytical IK as food pickup but targeting face position
- Slower approach trajectory (4 seconds vs 2 seconds for food pickup)
- Fork held roughly horizontal (j4 = 0.3 rad) for feeding

### Safety Features

| Safety | Trigger | Action |
|--------|---------|--------|
| Face lost (PRE_FEED) | No face for 3 seconds (30 frames) | Retract |
| Face lost (FEED) | No face for 1.5 seconds (15 frames) | Retreat to PRE_FEED |
| Mouth closes | `mouth_open` goes False during FEED | Pause, wait for re-open |
| Force collision | Force > 5N during face approach | Emergency retract |
| Mouth-open debounce | Requires 5 consecutive open frames | Prevents accidental feeding |
| Expression gating | Only proceeds when expression == "ready" | Pauses if eyes closed / face turned away |
| Manual override | `/feeding_override` topic | Bypasses face/mouth checks |

### Fusion Node Updates

- Subscribes to `/face_bearing` and `/face_detected` from face_node
- Publishes `/face_position_3d` (Point) — EMA-smoothed 3D face position in camera frame
- Updates `/mouth_distance` from face depth when face is detected

### Testing Face Detection

```bash
# Install MediaPipe
pip3 install mediapipe

# Build
cd ~/feeding_robot_ws/ros2_feedbot_ws
colcon build --symlink-install --packages-select feedbot_fusion feeding_robot
source install/setup.bash

# Test face_node standalone (camera must be running)
ros2 run feedbot_fusion face_node &
ros2 topic echo /face_detected --once
ros2 topic echo /mouth_open --once
ros2 topic echo /mouth_aspect_ratio --once
ros2 topic echo /face_expression --once
ros2 topic echo /face_bearing --once
```

### Calibrating Mouth Threshold

To find the optimal MAR threshold for your user:

```bash
# Watch raw MAR values while opening/closing mouth
ros2 topic echo /mouth_aspect_ratio

# Typical values:
#   Mouth closed: MAR ~ 0.05-0.15
#   Mouth slightly open: MAR ~ 0.2-0.3
#   Mouth wide open: MAR ~ 0.4-0.7
#   Default threshold: 0.3

# Override threshold at launch:
ros2 run feedbot_fusion face_node --ros-args -p mar_threshold:=0.25
```

### Full Autonomous Launch

```bash
ros2 launch feeding_robot hardware.launch.py
```

This now launches all nodes including face_node. The robot will:
1. Wait at home position
2. Auto-start when plate and food are both detected
3. Fork food using EKF-fused 3D position + analytical IK
4. Rotate to find user's face
5. Approach face using face-bearing IK (stops 5cm away)
6. Wait for mouth to open (5 consecutive frames)
7. Deliver food
8. Retract and auto-restart

### Complete ROS 2 Topic Map (with CV/AI)

| Topic | Type | Source | Purpose |
|-------|------|--------|---------|
| `/face_detected` | Bool | face_node | Face found in frame |
| `/face_center` | Point | face_node | Face position in pixels |
| `/face_bearing` | Point | face_node | Face bearing + depth |
| `/mouth_open` | Bool | face_node | Mouth open/closed |
| `/mouth_aspect_ratio` | Float64 | face_node | Raw MAR for tuning |
| `/face_expression` | String | face_node | "ready"/"not_ready"/"no_face" |
| `/mouth_ready_prediction` | Bool | face_node | Combined readiness signal |
| `/plate_detected` | Bool | vision_node | Plate found via shape analysis |
| `/face_position_3d` | Point | fusion_node | EMA-smoothed 3D face position |
| `/food_visible` | Bool | vision_node | Food detected |
| `/food_bearing` | Point | vision_node | Food bearing + depth |
| `/food_position_3d` | Point | fusion_node | EKF-fused 3D food position |
| `/spoon_force` | Float64 | teensy_bridge | Force sensor (N) |
| `/sonar_raw_cm` | Float64 | teensy_bridge | Ultrasonic distance (cm) |
| `/feeding_state` | String | feeding_fsm | Current FSM state |

### Key Files (CV/AI)

| File | Purpose |
|------|---------|
| `feedbot_fusion/face_node.py` | MediaPipe face/mouth detection, expression, depth |
| `feedbot_fusion/vision_node.py` | Food detection + plate detection |
| `feedbot_fusion/feeding_fsm_node.py` | Autonomous FSM with face tracking + mouth wait |
| `feedbot_fusion/fusion_node.py` | EKF fusion + face position pass-through |
| `feedbot_fusion/arima_ffnn_node.py` | ARIMA-FFNN prediction (mouth prediction removed) |
| `requirements.txt` | Added `mediapipe>=0.10.9` |

### Troubleshooting (CV/AI)

| Symptom | Cause | Fix |
|---------|-------|-----|
| `ModuleNotFoundError: mediapipe` | Not installed | `pip3 install mediapipe` |
| Face never detected | Camera blocked or face too far | Check `/feeding_robot/camera/image_raw` is publishing; move face within 0.5m |
| MAR always below threshold | Threshold too high or face at angle | Lower `mar_threshold` (try 0.2); face camera directly |
| Face node too slow (<3fps) | Pi 4 CPU load | Increase `process_every_n` to 4 or 5 |
| FSM auto-starts prematurely | Plate detection false positive | Adjust circularity threshold in `vision_node._detect_plate()` |
| Robot doesn't approach face | Face depth out of IK range | Face must be 0.15-0.40m from camera; check `/face_bearing` z value |
| Feed never triggers | Mouth-open debounce too strict | Reduce `MOUTH_OPEN_CONFIRM_FRAMES` in FSM (default 5) |
| Emergency retract during approach | Force sensor noise | Increase `FORCE_COLLISION_THRESHOLD` (default 5.0N) or check HX711 calibration |

---

## Keyboard Teleop — Manual Joint Control

A keyboard teleop node for manually moving the arm joint-by-joint.

### Usage

```bash
# With hardware.launch.py already running in another terminal:
source ~/feeding_robot_ws/ros2_feedbot_ws/install/setup.bash
ros2 run feedbot_fusion teleop_arm
```

### Controls

| Key | Action |
|-----|--------|
| W / S | Joint 1 (base yaw) +/- |
| E / D | Joint 2 (shoulder pitch) +/- |
| R / F | Joint 3 (elbow pitch) +/- |
| T / G | Joint 4 (feeder tilt) +/- |
| H | Home position (0, 0, 0, 0) |
| + / - | Increase / decrease step size |
| Q | Quit |

- Default step size: 0.05 rad (~2.9° per keypress)
- Uses the `arm_controller/follow_joint_trajectory` action interface
- Respects joint limits from the URDF
- Shows current joint angles after each keypress

**File:** `feedbot_fusion/feedbot_fusion/teleop_arm_node.py`

---

## Teensy Auto-Reconnect Behaviour

The Teensy bridge node (`teensy_bridge_node.py`) automatically handles connect/disconnect:

- On startup, attempts to connect to `/dev/teensy` (udev symlink)
- If the Teensy is **not plugged in**: logs a warning every **5 seconds** (not spamming)
- If the Teensy **disconnects mid-session** (USB cable pulled, I/O error): auto-reconnects on the next retry cycle
- If the Teensy is **plugged in later**: auto-connects within 5 seconds
- The robot operates normally without the Teensy — servos, camera, and face detection all work. Only force/sonar data is unavailable until the Teensy connects.

---

## Emergency Stop Button (Physical GPIO)

### Hardware Wiring

Connect a normally-open (NO) push button or mushroom-head E-Stop between GPIO 17 and GND:

```
Raspberry Pi 4:
  GPIO 17 (physical pin 11) ──── E-Stop Button ──── GND (physical pin 9)
```

No external resistor needed — the Pi's internal pull-up resistor is used.

### How It Works

1. **Button pressed** → `/emergency_stop` topic publishes `True`
2. **Immediately** → FSM enters `EMERGENCY_STOP` state, arm freezes at current position
3. **After 2 seconds** → Dynamixel torque disabled (arm goes limp)
4. **Button released** → Torque re-enabled, FSM returns to `WAITING`

### Dependencies

```bash
# Install GPIO library (on the Pi)
pip3 install RPi.GPIO
# or for modern kernel interface:
sudo apt install python3-gpiod gpiod
```

### Launch

The E-Stop node starts automatically with the hardware launch:

```bash
ros2 launch feeding_robot hardware.launch.py
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gpio_pin` | 17 | BCM GPIO pin number |
| `hold_before_disable` | 2.0 | Seconds to hold position before torque off |
| `dynamixel_port` | `/dev/dynamixel` | Dynamixel serial port |
| `baud_rate` | 1000000 | Dynamixel baud rate |

### Software E-Stop Fallback

If no physical button is wired, you can trigger E-Stop from any terminal:

```bash
# Activate E-Stop
ros2 topic pub /emergency_stop_sw std_msgs/msg/Bool "{data: true}" --once

# Release E-Stop
ros2 topic pub /emergency_stop_sw std_msgs/msg/Bool "{data: false}" --once
```

### Testing

```bash
# Monitor E-Stop state
ros2 topic echo /emergency_stop

# Monitor FSM state (should show EMERGENCY_STOP when pressed)
ros2 topic echo /feeding_state
```

### FSM State: EMERGENCY_STOP

When E-Stop is active:
- All FSM state transitions are blocked
- Arm holds at the position it was in when E-Stop was pressed
- After `hold_before_disable` seconds, torque is disabled via Dynamixel SDK
- On release: torque re-enabled, FSM returns to `WAITING`

### Key Files

| File | Purpose |
|------|---------|
| `feedbot_fusion/estop_node.py` | GPIO monitor, torque control via Dynamixel SDK |
| `feedbot_fusion/feeding_fsm_node.py` | EMERGENCY_STOP state, `/emergency_stop` subscriber |

---

## Camera Calibration

### Quick Setup (Default Calibration)

Create a default calibration file using Pi Camera V2.1 specs (62° HFOV, fx=fy=554.25 at 640x480):

```bash
mkdir -p ~/.ros/camera_info
cat > ~/.ros/camera_info/mmal_service_16.1.yaml << 'EOF'
image_width: 640
image_height: 480
camera_name: mmal_service_16.1
camera_matrix:
  rows: 3
  cols: 3
  data: [554.25, 0, 320, 0, 554.25, 240, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0, 0, 0, 0, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [554.25, 0, 320, 0, 0, 554.25, 240, 0, 0, 0, 1, 0]
EOF
```

This stops the calibration warning and provides approximate intrinsics for the robot's operation.

### Proper Checkerboard Calibration (Optional)

For more accurate results (requires a monitor connected to the Pi):

```bash
# 1. Install calibration tool
sudo apt install ros-humble-camera-calibration

# 2. Print a 9x6 checkerboard (25mm squares), tape flat on cardboard

# 3. Start hardware launch in Terminal 1
ros2 launch feeding_robot hardware.launch.py

# 4. Run calibrator in Terminal 2
ros2 run camera_calibration cameracalibrator \
  --size 9x6 \
  --square 0.025 \
  --no-service-check \
  --ros-args -r image:=/feeding_robot/camera/image_raw \
  -r camera:=/feeding_robot/camera

# 5. Move checkerboard around — cover all corners, angles, distances
# 6. When X/Y/Size/Skew bars are green → click CALIBRATE → SAVE → COMMIT
```

---

## Digital Twin (RViz2)

The hardware launch can optionally start RViz2 as a real-time digital twin of the robot arm.

### Usage

```bash
# With monitor connected to Pi:
ros2 launch feeding_robot hardware.launch.py digital_twin:=true

# Headless (default, no RViz):
ros2 launch feeding_robot hardware.launch.py
```

### What It Shows

- 3D robot model moving in real-time (from `/joint_states`)
- TF coordinate frames for all links and sensors
- Grid reference plane
- Plate mesh in front of the robot (matches real setup)

### RViz Config

Pre-configured in `feeding_robot/config/digital_twin.rviz`:
- Fixed Frame: `world`
- RobotModel from `/robot_description`
- TF frames enabled
- Orbit camera view centered on the robot

### Plate in URDF

A plate mesh (`plate_printing.stl`) is included as a fixed link in front of the robot base, matching the real-world setup:

```xml
<joint name="plate_joint" type="fixed">
  <parent link="world"/>
  <child link="plate_link"/>
  <origin xyz="0.15 0.0 -0.02" rpy="0 0 0"/>
</joint>
```

Adjust `xyz` values in `feeding_robot_core.xacro` if the plate position doesn't match your physical setup.

---

## Dynamixel PID Tuning

The Dynamixel XM430-W350R servos use internal PID position control. Gains are configured in `feeding_robot_ros2_control.xacro` via a xacro macro applied to all 4 servos:

### Current Values

| Parameter | Value | Effect |
|-----------|-------|--------|
| Position P Gain | 400 | Proportional — response stiffness |
| Position I Gain | 20 | Integral — steady-state error correction |
| Position D Gain | 150 | Derivative — damping/overshoot reduction |
| Profile Velocity | 60 | Max velocity (0.229 rpm units) |
| Profile Acceleration | 15 | Acceleration ramp (214.577 rev/min² units) |

### Tuning Guide

| Symptom | Fix |
|---------|-----|
| Overshoot/oscillation | Lower P (try 300), lower I (try 10), raise D (try 200) |
| Too sluggish/slow | Raise P (try 500), raise Profile Velocity (try 100) |
| Jerky motion | Lower Profile Acceleration (try 10), raise trajectory duration in teleop |
| Can't hold position under load | Raise I (try 30-50), raise P (try 500) |

### Changing PID on the Fly (Without Rebuild)

```bash
# Write directly to servo register via Python
python3 -c "
from dynamixel_sdk import *
port = PortHandler('/dev/dynamixel')
packet = PacketHandler(2.0)
port.openPort()
port.setBaudRate(1000000)
for sid in [11, 12, 13, 14]:
    packet.write2ByteTxRx(port, sid, 80, 400)   # P Gain (addr 80)
    packet.write2ByteTxRx(port, sid, 82, 20)    # I Gain (addr 82)
    packet.write2ByteTxRx(port, sid, 84, 150)   # D Gain (addr 84)
    packet.write4ByteTxRx(port, sid, 112, 60)   # Profile Velocity (addr 112)
    packet.write4ByteTxRx(port, sid, 108, 15)   # Profile Acceleration (addr 108)
    print(f'ID {sid}: PID updated')
port.closePort()
"
```

---

## Object Detection (MediaPipe)

### Detection Backends

The vision_node auto-selects the best available backend:

| Backend | Platform | Speed | Objects | Selection |
|---------|----------|-------|---------|-----------|
| jetson-inference | Jetson Orin/Nano (GPU) | ~30fps | COCO 91 classes | Auto on Jetson |
| MediaPipe EfficientDet-Lite0 | Pi 4/5 (CPU) | ~5-8fps | COCO 80 classes | Auto on Pi |
| HSV colour | Any (fallback) | ~30fps | 6 fruit types only | Fallback if no ML |

### MediaPipe Model

- **Model:** EfficientDet-Lite0 float32 (~4MB)
- **Auto-downloads** to `~/.mediapipe/efficientdet_lite0_float32.tflite` on first run
- **Manual download** if auto-download fails:

```bash
mkdir -p ~/.mediapipe
wget -O ~/.mediapipe/efficientdet_lite0_float32.tflite \
  "https://storage.googleapis.com/mediapipe-models/object_detector/efficientdet_lite0/float32/latest/efficientdet_lite0_float32.tflite"
```

### Detection Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `detection_method` | `auto` | `auto`, `mediapipe`, `ml`, `hsv` |
| `ml_confidence` | 0.25 | Minimum detection confidence (0.0-1.0) |
| `process_every_n` | 3 | Process every Nth camera frame |

### Detected Object Categories

| Category | Objects | Topic |
|----------|---------|-------|
| Food | apple, banana, orange, sandwich, broccoli, carrot, pizza, donut, cake, hot_dog | `/food_visible`, `/food_type` |
| Plate/Bowl | bowl, cup, dining table, plate | `/plate_detected` |
| Utensils | fork, knife, spoon | `/objects_detected` |

### Monitoring

```bash
# All detected objects (comma-separated with confidence)
ros2 topic echo /objects_detected

# Food detection
ros2 topic echo /food_visible --once
ros2 topic echo /food_type --once

# Plate detection
ros2 topic echo /plate_detected --once
```

---

## Complete Hardware Launch — All Nodes

`ros2 launch feeding_robot hardware.launch.py` starts the following nodes:

| Node | Package | Purpose |
|------|---------|---------|
| `ros2_control_node` | controller_manager | Dynamixel hardware interface (4 joints) |
| `robot_state_publisher` | robot_state_publisher | TF tree from URDF |
| `joint_state_broadcaster` | controller_manager | Publishes `/joint_states` |
| `arm_controller` | controller_manager | Joint trajectory controller (100 Hz) |
| `feeding_camera` | v4l2_camera | Pi Camera → `/feeding_robot/camera/image_raw` |
| `vision_node` | feedbot_fusion | Object/food/plate detection (MediaPipe) |
| `teensy_bridge` | feedbot_fusion | Teensy serial → force + sonar topics |
| `sonar_bridge` | feedbot_fusion | Routes sonar to plate/mouth distance |
| `face_node` | feedbot_fusion | MediaPipe face/mouth detection |
| `estop_node` | feedbot_fusion | GPIO E-Stop button monitor |
| `rviz2` | rviz2 | Digital twin (only with `digital_twin:=true`) |

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `port_name` | `/dev/dynamixel` | Dynamixel USB port |
| `teensy_port` | `/dev/teensy` | Teensy serial port |
| `camera_device` | `/dev/video0` | Camera device path |
| `use_meshes` | `true` | Use STL mesh files in URDF |
| `digital_twin` | `false` | Launch RViz2 as digital twin |

---

## Complete ROS 2 Topic Map

| Topic | Type | Source | Purpose |
|-------|------|--------|---------|
| `/joint_states` | JointState | joint_state_broadcaster | Joint positions/velocities |
| `/feeding_robot/camera/image_raw` | Image | v4l2_camera | Camera feed (640x480) |
| `/feeding_robot/camera/camera_info` | CameraInfo | v4l2_camera | Camera intrinsics |
| `/food_visible` | Bool | vision_node | Food detected flag |
| `/food_center` | Point | vision_node | Food position in pixels |
| `/food_type` | String | vision_node | Food class name |
| `/food_bearing` | Point | vision_node | Food bearing + depth |
| `/detected_fruits` | Float64MultiArray | vision_node | All food detections |
| `/plate_detected` | Bool | vision_node | Plate/bowl detected |
| `/objects_detected` | String | vision_node | All detected objects |
| `/face_detected` | Bool | face_node | Face found |
| `/face_center` | Point | face_node | Face position in pixels |
| `/face_bearing` | Point | face_node | Face bearing + depth |
| `/mouth_open` | Bool | face_node | Mouth open/closed |
| `/mouth_aspect_ratio` | Float64 | face_node | Raw MAR value |
| `/face_expression` | String | face_node | "ready"/"not_ready"/"no_face" |
| `/mouth_ready_prediction` | Bool | face_node | Combined readiness |
| `/spoon_force` | Float64 | teensy_bridge | Force sensor (N) |
| `/spoon/wrench` | WrenchStamped | teensy_bridge | Force as wrench |
| `/feeding_robot/ultrasonic/range` | Range | teensy_bridge | Sonar range |
| `/sonar_raw_cm` | Float64 | teensy_bridge | Raw sonar (cm) |
| `/sonar_plate_distance` | Float64 | sonar_bridge | Distance to plate (cm) |
| `/sonar_mouth_distance` | Float64 | sonar_bridge | Distance to mouth (cm) |
| `/emergency_stop` | Bool | estop_node | E-Stop state |
| `/feeding_state` | String | feeding_fsm | Current FSM state |

---

## Complete File Reference

| File | Purpose |
|------|---------|
| **feeding_robot package** | |
| `description/feeding_robot.urdf.xacro` | Top-level URDF (includes all sub-xacros) |
| `description/feeding_robot_core.xacro` | Links, joints, meshes, plate, sensor mounts |
| `description/feeding_robot_ros2_control.xacro` | Dynamixel hardware interface, servo PID, GPIO configs |
| `description/materials.xacro` | Material/color definitions |
| `description/gazebo_plugins.xacro` | Gazebo sensor plugins (sim only) |
| `config/feeding_robot_controllers.yaml` | Controller manager (trajectory controller, 100 Hz) |
| `config/digital_twin.rviz` | RViz2 display configuration |
| `launch/hardware.launch.py` | Real hardware launch (all nodes) |
| `launch/gazebo.launch.py` | Simulation launch |
| `meshes/visual/` | STL meshes (arm links, frontface, camera, sonar, forceSensor, plate) |
| **feedbot_fusion package** | |
| `face_node.py` | MediaPipe face/mouth detection |
| `vision_node.py` | Object/food/plate detection (MediaPipe/HSV) |
| `feeding_fsm_node.py` | Autonomous FSM (10 states including EMERGENCY_STOP) |
| `fusion_node.py` | 6-state EKF sensor fusion + face position |
| `teensy_bridge_node.py` | Teensy serial bridge (force + sonar) |
| `sonar_bridge_node.py` | Routes sonar by feeding state |
| `estop_node.py` | GPIO E-Stop + Dynamixel torque control |
| `teleop_arm_node.py` | Keyboard joint control |
| `force_node.py` | Force filtering (moving average) |
| `arima_ffnn_node.py` | ARIMA-FFNN predictive controller |
| `fuzzy_controller_node.py` | Fuzzy force/angle regulation |
| `mouth_animator_node.py` | Simulated jaw animation (Gazebo only) |
| **firmware** | |
| `firmware/teensy_sensors.ino` | Teensy firmware (HX711 + HC-SR04) |
| **config** | |
| `requirements.txt` | Python dependencies |
| `/etc/udev/rules.d/99-feedbot.rules` | USB device symlinks |
| `~/.ros/camera_info/mmal_service_16.1.yaml` | Camera calibration |

---

## FSM States (Complete)

```
WAITING ──[plate+food detected OR spacebar]──> IDLE
IDLE ──[food_visible]──> DETECT_FOOD
DETECT_FOOD ──[food confirmed]──> LOCATE_FOOD
LOCATE_FOOD ──[EKF stable + IK computed]──> COLLECT_FOOD
COLLECT_FOOD ──[force > 2N]──> DETECT_PATIENT
DETECT_PATIENT ──[face_detected + expression=="ready"]──> PRE_FEED
PRE_FEED ──[face-approach IK + at target + safe]──> FEED
FEED ──[mouth_open for 5 consecutive frames]──> RETRACT
RETRACT ──[at target]──> WAITING (auto-restart)

EMERGENCY_STOP ── (any state, triggered by E-Stop button or /emergency_stop topic)
  → Freezes arm at current position
  → Disables torque after 2 seconds
  → Returns to WAITING on release
```

---

## ARIMA-FFNN vs PID Performance Comparison

### Overview

The robot supports two control modes for A/B performance comparison:

| Mode | Description | What's Active |
|------|-------------|---------------|
| `pid_only` | Dynamixel internal PID only (baseline) | Servo PID control, no predictions |
| `pid_arima` | PID + ARIMA-FFNN feedforward | Servo PID + predictive compensation |

### How ARIMA Feedforward Works

```
ARIMA-FFNN predicts next joint positions → feedforward offset computed →
trajectory goal pre-compensated → PID starts correcting earlier → less lag
```

The feedforward offset is: `offset[i] = gain * (predicted[i] - current[i])`
This anticipates where the joint will be, reducing reactive delay.

### Running the Benchmark

**Step 1: PID-only baseline (10 feeding cycles)**
```bash
ros2 launch feeding_robot hardware.launch.py benchmark:=true mode:=pid_only
# Run teleop or autonomous feeding for several cycles
# Ctrl+C when done — CSV saved to ~/feedbot_benchmark/
```

**Step 2: PID+ARIMA mode (10 feeding cycles)**
```bash
ros2 launch feeding_robot hardware.launch.py benchmark:=true mode:=pid_arima
# Run the same trajectory/cycles
# Ctrl+C — CSV saved to ~/feedbot_benchmark/
```

**Step 3: Compare results**
```bash
ls ~/feedbot_benchmark/
# pid_only_20260410_093700.csv
# pid_arima_20260410_094200.csv
```

### Live Metrics Monitoring

```bash
# Watch real-time metrics (JSON)
ros2 topic echo /benchmark/metrics
```

Output example:
```json
{
  "mode": "pid_arima",
  "base_y_joint": {"rmse": 0.012, "overshoot": 0.035, "settle_time": 0.8, "avg_jerk": 12.5},
  "lower_z_joint": {"rmse": 0.015, "overshoot": 0.042, "settle_time": 1.1, "avg_jerk": 15.2},
  "upper_z_joint": {"rmse": 0.010, "overshoot": 0.028, "settle_time": 0.6, "avg_jerk": 10.1},
  "feeder_joint": {"rmse": 0.008, "overshoot": 0.020, "settle_time": 0.5, "avg_jerk": 8.3},
  "avg_cycle_time": 12.5,
  "cycles": 5
}
```

### Metrics Recorded

| Metric | Description | Unit | Better = |
|--------|-------------|------|----------|
| RMSE | Root mean square tracking error | rad | Lower |
| Overshoot | Max deviation beyond target | rad | Lower |
| Settling time | Time to reach within 0.02 rad of target | seconds | Lower |
| Avg jerk | Average jerk (smoothness) | rad/s³ | Lower |
| Prediction error | ARIMA predicted vs actual | rad | Lower |
| Cycle time | Total feeding cycle duration | seconds | Lower |

### CSV Format

```
time,j1_target,j1_actual,j1_error,j1_predicted,j1_pred_error,...,fsm_state
0.020,0.0000,-0.0012,-0.0012,0.0015,0.0027,IDLE
0.040,0.0000,-0.0008,-0.0008,0.0010,0.0018,IDLE
...
```

### Key Files

| File | Purpose |
|------|---------|
| `feedbot_fusion/benchmark_node.py` | Records metrics to CSV, publishes `/benchmark/metrics` |
| `feedbot_fusion/arima_feedforward_node.py` | ARIMA feedforward compensator (action proxy) |
| `feedbot_fusion/arima_ffnn_node.py` | ARIMA(3,1,1) + FFNN(10→16→8→1) predictive model |
| `feedbot_fusion/feeding_fsm_node.py` | `use_arima_feedforward` parameter for in-FSM compensation |

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `benchmark` | `false` | Enable benchmark recording |
| `mode` | `pid_only` | `pid_only` or `pid_arima` |

---

## Quick Start Checklist

```bash
# 1. SSH into Pi
ssh ss@<pi-ip>

# 2. Set permissions (if first boot)
sudo chmod 666 /dev/ttyACM*
sudo udevadm control --reload-rules && sudo udevadm trigger

# 3. Build
cd ~/feeding_robot_ws/ros2_feedbot_ws
colcon build --symlink-install
source install/setup.bash

# 4. Launch (headless)
ros2 launch feeding_robot hardware.launch.py

# 4b. Launch with digital twin (monitor connected)
ros2 launch feeding_robot hardware.launch.py digital_twin:=true

# 5. Teleop (separate terminal)
source ~/feeding_robot_ws/ros2_feedbot_ws/install/setup.bash
ros2 run feedbot_fusion teleop_arm

# 6. Monitor sensors (separate terminal)
source ~/feeding_robot_ws/ros2_feedbot_ws/install/setup.bash
ros2 topic echo /face_detected --once
ros2 topic echo /food_visible --once
ros2 topic echo /objects_detected
ros2 topic echo /spoon_force --once
ros2 topic echo /joint_states --once
```