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

```
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

## ML Food Detection (jetson-inference)

The vision node uses **NVIDIA jetson-inference** for ML-based object detection on
Jetson hardware, with HSV colour segmentation as fallback on dev PCs / Gazebo.

### Installing jetson-inference

```bash
# On Jetson Orin (one-time setup)
sudo apt-get install git cmake libpython3-dev python3-numpy
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build && cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig
```

### Detection Method

The vision node auto-detects whether jetson-inference is available:

| Environment | Detection Method | Notes |
|-------------|-----------------|-------|
| Jetson Orin (real hardware) | **ML** (SSD-Mobilenet-v2 via detectNet) | Detects COCO food classes at ~30 FPS |
| Dev PC / Gazebo simulation | **HSV** colour segmentation (fallback) | Works with Gazebo's solid-colour objects |

**Force a specific method via ROS parameter:**

```bash
# Force ML detection (will warn if jetson-inference not installed)
ros2 run feedbot_fusion vision_node --ros-args -p detection_method:=ml

# Force HSV fallback
ros2 run feedbot_fusion vision_node --ros-args -p detection_method:=hsv

# Auto-detect (default)
ros2 run feedbot_fusion vision_node --ros-args -p detection_method:=auto
```

### COCO Food Classes Detected

The SSD-Mobilenet-v2 model detects these food items from the COCO dataset:

| Class ID | Name | Class ID | Name |
|----------|------|----------|------|
| 52 | banana | 53 | apple |
| 54 | sandwich | 55 | orange |
| 56 | broccoli | 57 | carrot |
| 58 | hot_dog | 59 | pizza |
| 60 | donut | 61 | cake |

### Tuning ML Confidence

```bash
# Adjust confidence threshold (default: 0.35)
ros2 run feedbot_fusion vision_node --ros-args -p ml_confidence:=0.25

# Use a different model
ros2 run feedbot_fusion vision_node --ros-args -p ml_model:=ssd-inception-v2
```

## Standalone Perception Demo (No ROS Required)

```bash
python ros2_feedbot_ws/scripts/perception_demo.py
```

- **With display** (Windows/desktop Linux): Opens a live OpenCV window with interactive keys (`q` quit, `o` outlier storm, `s` sonar failure)
- **Headless** (SSH/Jetson without `$DISPLAY`): Saves a 300-frame video to `perception_demo_output.avi`, prints metrics to console every 30 frames, auto-enables outlier storm at frames 150-210 to demonstrate Kalman filter resilience
