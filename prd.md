# Feeding Robot ARIMA-FFNN — Project Reference Document (PRD)

## Overview

An assistive feeding robot for patients who cannot feed themselves. A **4-DOF robotic arm** mounted beside a feeding table detects food on a plate using computer vision, picks it up with a fork/spoon, and delivers it to the patient's mouth. Built on **ROS 2 Humble** with **Gazebo Fortress** simulation and **NVIDIA Jetson Orin** for real-time ML inference.

**Repository:** `git@github.com:ss48/Feeding-Robot-ARIMA-FFNN.git`

---

## Architecture

```
Camera → [vision_node]    → food_visible, food_center, food_type, food_bearing
                                  ↓
Sonar  → [sonar_bridge]   → sonar_plate_distance, sonar_mouth_distance
                                  ↓
Force  → [force_node]     → spoon_force
                                  ↓
          [fusion_node]    → food_position_3d (fused 3D), plate_distance,
          (6-state EKF)      mouth_distance, fusion_confidence, sensor_health
                                  ↓
          [arima_ffnn]     → predicted_state, prediction_error
                                  ↓
          [fuzzy_ctrl]     → target_force, target_angle, feeding_safe
                                  ↓
          [feeding_fsm]    → arm_controller commands, feeding_state
          (IK from 3D pos)   WAITING→IDLE→DETECT_FOOD→LOCATE_FOOD→
                              COLLECT_FOOD→DETECT_PATIENT→PRE_FEED→
                              FEED→RETRACT→WAITING
```

---

## Packages (7 total)

| Package | Type | Location | Purpose |
|---------|------|----------|---------|
| `feedbot_description` | ament_cmake | `ros2_feedbot_ws/src/feedbot_description/` | Robot URDF (STL meshes), Gazebo launch, controllers, RViz |
| `feedbot_fusion` | ament_python | `ros2_feedbot_ws/src/feedbot_fusion/` | All intelligence nodes: vision, fusion, FSM, ARIMA-FFNN, fuzzy |
| `feedbot_moveit_config` | ament_cmake | `ros2_feedbot_ws/feedbot_moveit_config/` | MoveIt2 motion planning config |
| `feedbot_sensors` | ament_python | `ros2_feedbot_ws/feedbot_sensors/` | Sensor node stubs |
| `feeding_robot` | ament_cmake | `feeding_robot/` (repo root) | Alternative primitive-shapes URDF + feeding_table.sdf world |
| `pid_logger` | ament_python | `ros2_feedbot_ws/src/pid_logger/` | PID logging utility |
| `cam_motion_light` | experimental | `ros2_feedbot_ws/cam_motion_light/` | Camera/lighting experiments |

### Important: Dual workspace issue

The Jetson has **two colcon install directories**:
- `/home/shab/feeding_robot_ws/install/` — **top-level** (Gazebo reads from here via `$(find feedbot_description)`)
- `/home/shab/feeding_robot_ws/ros2_feedbot_ws/install/` — inner workspace

When editing URDF or controller configs, you must update the **top-level install** or changes won't take effect in Gazebo. The `$(find feedbot_description)` resolves to the top-level path.

---

## Robot Description (feedbot_description)

### URDF: `my_robot.urdf.xacro`

**4-DOF arm with Dynamixel XM430-W350R servos:**

| Joint | Name | Axis | Limits (rad) | Function |
|-------|------|------|-------------|----------|
| 1 | `joint1` | Z (yaw) | -3.05 to +3.05 | Base rotation |
| 2 | `joint2` | Y (pitch) | -1.57 to +1.57 | Shoulder up/down |
| 3 | `joint3` | Y (pitch) | -2.09 to +1.31 | Elbow up/down |
| 4 | `joint4` | Y (pitch) | -1.57 to +2.01 | Feeder head tilt |

**Links:** base → link1 → link2 → link3 → feeder_head (all with STL meshes at 0.01 scale)

**Fixed links (part of URDF):** table_link, plate_link, apple_link, strawberry_link, banana_link, orange_link, mouth_open_link (patient face)

**Sensors in URDF:**
- Camera: on `feeder_head` at (0.02, 0, 0.03), 640x480, 30Hz, 62° FOV
- Ultrasonic: on `feeder_head` at (0.02, 0, 0.02), gpu_lidar, 20Hz
- Force/torque: on `joint4`, 50Hz

### ros2_control Configuration

The URDF defines the hardware interface:
```xml
<ros2_control name="FeedbotSystem" type="system">
  <hardware>
    <plugin>ign_ros2_control/IgnitionSystem</plugin>
  </hardware>
  <joint name="joint1">
    <command_interface name="effort"/>  <!-- or "position" -->
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- ... joint2, joint3, joint4 same pattern -->
</ros2_control>
```

**Gazebo plugin:**
```xml
<gazebo>
  <plugin filename="libign_ros2_control-system.so"
          name="ign_ros2_control::IgnitionROS2ControlPlugin">
    <parameters>$(find feedbot_description)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

### Controller Options (controllers.yaml)

**Effort interface + JointTrajectoryController with PID** (proven to move arm):
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4]
    command_interfaces: [effort]
    state_interfaces: [position, velocity]
    open_loop_control: true
    gains:
      joint1: {p: 180.0, d: 8.0, i: 0.02}
      joint2: {p: 150.0, d: 10.0, i: 0.15}
      joint3: {p: 120.0, d: 8.0, i: 0.05}
      joint4: {p: 60.0, d: 6.0, i: 0.02}
```

**Position interface + ForwardCommandController** (simpler but may not work on all Gazebo versions):
```yaml
arm_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4]
    interface_name: position
```

### Known Issues with Controllers

| Issue | Cause | Solution |
|-------|-------|----------|
| `pid_controller/PidController` not found | Not available in ROS 2 Humble (added in Iron) | Use `joint_trajectory_controller` with effort + PID gains |
| `ign_ros2_control plugin got renamed` | Humble uses `ign_` names, newer uses `gz_` | Use `ign_ros2_control/IgnitionSystem` + `libign_ros2_control-system.so` |
| Position interface doesn't move arm | `IgnitionSystem` position interface may not work on Fortress | Use effort interface with PID gains instead |
| Goal rejected by JointTrajectoryController | Sim-time / start-state validation | Add `open_loop_control: true` to controller params |
| Hardware plugin names must match | URDF hardware and Gazebo plugin must both use `ign_` or both `gz_` | Keep consistent: both `ign_ros2_control` |
| Controller reads from wrong install dir | Two colcon workspaces, top-level takes priority | Edit files at `/home/shab/feeding_robot_ws/install/feedbot_description/share/feedbot_description/` |

---

## Feeding System Nodes (feedbot_fusion)

### Launch
```bash
ros2 launch feedbot_fusion feeding_system.launch.py
```

### Node Details

**vision_node.py** — Food detection
- ML mode (Jetson): `detectNet` with SSD-Mobilenet-v2 (COCO, 10+ food classes)
- HSV fallback (Gazebo): colour segmentation for apple, strawberry, banana, grape, orange, kiwi
- Publishes: `/food_visible`, `/food_center`, `/food_type`, `/food_bearing`, `/detected_fruits`
- Camera model: pinhole, h_fov=1.047 rad, 640x480
- Detection method parameter: `hsv` (default), `auto`, `ml`

**fusion_node.py** — 6-state EKF sensor fusion
- State: `[food_x, food_y, food_z, plate_distance, mouth_distance, force]`
- Fuses: camera bearing + monocular depth + ultrasonic range + force + joints + ARIMA predictions
- Triple update cycle: camera/force/joints → ultrasonic depth → refined bearing
- Publishes: `/food_position_3d`, `/plate_distance`, `/mouth_distance`, `/fusion_confidence`, `/sensor_health`

**feeding_fsm_node.py** — State machine + IK
- States: WAITING → IDLE → DETECT_FOOD → LOCATE_FOOD → COLLECT_FOOD → DETECT_PATIENT → PRE_FEED → FEED → RETRACT → WAITING
- Spacebar triggered via `/feeding_start`
- Analytical IK converts fused 3D food position to joint angles
- Publishes to `/arm_controller/commands` (ForwardCommandController) or `/arm_controller/joint_trajectory` (JointTrajectoryController)

**arima_ffnn_node.py** — Hybrid predictive controller
- Time-series decomposition: Trend + Seasonality + Residual
- ARIMA (linear) + FFNN (nonlinear) combined prediction
- Online training via backpropagation

**fuzzy_controller_node.py** — Fuzzy logic control
- Rules: plate_distance × food_type → target_force + target_angle
- Publishes: `/target_force`, `/target_angle`, `/feeding_safe`

**force_node.py** — Force filtering
- Moving average filter (10-frame buffer) on `/spoon/wrench`
- Publishes: `/spoon_force`

**sonar_bridge_node.py** — Ultrasonic conversion
- Converts Gazebo LaserScan → distance Float64
- Routes by FSM state: DETECT_FOOD → `/sonar_plate_distance`, FEED → `/sonar_mouth_distance`

**mouth_animator_node.py** — Patient jaw animation (Gazebo only)
- 4-second cycle on `/patient_head/jaw_cmd`

---

## MoveIt2 Configuration (feedbot_moveit_config)

**SRDF** (`feedbot.srdf`):
- Group `arm`: joint1-4, chain base→feeder_head
- Virtual joint: `world` → `base_root` (fixed)

**Kinematics** (`kinematics.yaml`):
- Solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`

**MoveIt Controllers** (`moveit_controllers.yaml`):
- Uses `arm_controller` with `FollowJointTrajectory` action
- Requires `joint_trajectory_controller` (not `forward_command_controller`)

**Launch:**
```bash
# Standalone demo (includes robot_state_publisher + move_group + RViz)
ros2 launch feedbot_moveit_config demo.launch.py

# Or integrated into Gazebo launch (after code update):
ros2 launch feedbot_description gazebo.launch.py
# This now includes move_group node + MoveIt RViz panel
```

---

## Gazebo Simulation

### Launch
```bash
ros2 launch feedbot_description gazebo.launch.py
```

This launches: Gazebo Fortress + robot spawn + sensor bridges + controllers + RViz

### Sensor Bridges (ros_gz_bridge)
```
/clock                              — Gazebo → ROS (sim time)
/feeding_robot/camera/image_raw     — Gazebo → ROS (camera)
/feeding_robot/ultrasonic/scan      — Gazebo → ROS (lidar/ultrasonic)
/spoon/wrench                       — Gazebo → ROS (force/torque)
```

### Scene Objects (in URDF, not SDF world)
The `feedbot_description` URDF includes table, plate, food, and patient face as fixed links. Gazebo uses `empty.world`. The `feeding_robot` package has a separate `feeding_table.sdf` world with its own objects — do not mix them to avoid duplicates.

### Gazebo Fortress Material Colors
Classic `Gazebo/Red` syntax does NOT work in Fortress. Use:
```xml
<gazebo reference="apple_link">
  <visual>
    <material>
      <ambient>0.8 0.12 0.08 1</ambient>
      <diffuse>0.8 0.12 0.08 1</diffuse>
      <specular>0.4 0.3 0.3 1</specular>
    </material>
  </visual>
</gazebo>
```

---

## Computer Vision — jetson-inference

### ML Detection (Jetson Orin)
- Uses NVIDIA `detectNet` with SSD-Mobilenet-v2 (COCO-trained)
- Runs on GPU via TensorRT for real-time 30 FPS inference
- Detects 10+ food classes: apple(53), banana(52), orange(55), sandwich(54), broccoli(56), carrot(57), hot_dog(58), pizza(59), donut(60), cake(61)
- Confidence threshold: 0.35

### HSV Fallback (Gazebo / dev PC)
- Used when `jetson-inference` is not available
- Colour segmentation with predefined HSV ranges per fruit type

### Installation (Jetson Orin / JetPack 6.x)
```bash
sudo apt-get install cuda-nvcc-12-6 cuda-cudart-dev-12-6
cd ~
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
cd jetson-inference && mkdir build && cd build
export CUDA_HOME=/usr/local/cuda-12.6
cmake -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.6 ../
make -j$(nproc)
sudo make install && sudo ldconfig
```

### Known Issue: TensorRT 10.3 + Legacy Models
JetPack 6.x ships TensorRT 10.3 which does NOT support legacy UFF/Caffe formats. The default SSD-Mobilenet-v2 uses `.uff` and will fail. Workaround: use HSV detection (default) or provide an ONNX model.

---

## Deployment on Jetson Orin

### Initial Setup
```bash
git clone git@github.com:ss48/Feeding-Robot-ARIMA-FFNN.git ~/feeding_robot_ws
cd ~/feeding_robot_ws
chmod +x jetson_setup.sh run_feeding.sh

# CRITICAL: Copy feeding_robot package into workspace
cp -r feeding_robot ros2_feedbot_ws/src/feeding_robot

./jetson_setup.sh
source ~/.bashrc
```

### Running

```bash
# Quick start (simulation)
./run_feeding.sh sim

# Or manual launch:
# Terminal 1: Gazebo + robot + controllers
ros2 launch feedbot_description gazebo.launch.py

# Terminal 2 (after ~12s): All feeding nodes
ros2 launch feedbot_fusion feeding_system.launch.py

# Terminal 3: Trigger feeding cycle
ros2 topic pub /feeding_start std_msgs/msg/Bool "{data: true}" --once
# Or press SPACE in run_feeding.sh terminal
```

### Modes (run_feeding.sh)
| Mode | Command | Description |
|------|---------|-------------|
| `sim` | `./run_feeding.sh sim` | Gazebo + all nodes |
| `real` | `./run_feeding.sh real` | Hardware mode (servos, camera, force sensor) |
| `predict` | `./run_feeding.sh predict` | ARIMA-FFNN node only |
| `fsm` | `./run_feeding.sh fsm` | FSM node only |
| `nodes` | `./run_feeding.sh nodes` | Individual nodes with logs |

---

## Key ROS 2 Topics

### Vision
| Topic | Type | Source |
|-------|------|--------|
| `/food_visible` | Bool | vision_node |
| `/food_center` | Point | vision_node (x=px, y=py, z=area) |
| `/food_type` | String | vision_node / FSM |
| `/food_bearing` | Point | vision_node (x=bearing_h, y=bearing_v, z=depth) |

### Sensor Fusion
| Topic | Type | Source |
|-------|------|--------|
| `/food_position_3d` | Point | fusion_node (metres) |
| `/plate_distance` | Float64 | fusion_node (cm) |
| `/mouth_distance` | Float64 | fusion_node (cm) |
| `/fusion_confidence` | Float64 | fusion_node (0-1) |
| `/sensor_health` | Float64MultiArray | fusion_node (6 values) |

### Control
| Topic | Type | Source |
|-------|------|--------|
| `/arm_controller/commands` | Float64MultiArray | FSM (ForwardCommandController) |
| `/arm_controller/joint_trajectory` | JointTrajectory | FSM (JointTrajectoryController) |
| `/feeding_state` | String | FSM |
| `/feeding_start` | Bool | Spacebar trigger |
| `/target_force` | Float64 | fuzzy_controller |
| `/target_angle` | Float64 | fuzzy_controller |

### Raw Sensors
| Topic | Type | Source |
|-------|------|--------|
| `/feeding_robot/camera/image_raw` | Image | Gazebo bridge (~30Hz) |
| `/feeding_robot/ultrasonic/scan` | LaserScan | Gazebo bridge (~20Hz) |
| `/spoon/wrench` | WrenchStamped | Gazebo bridge (~50Hz) |
| `/joint_states` | JointState | joint_state_broadcaster |

---

## Calibration Procedures

### Joint Poses (feeding_fsm_node.py)
Use `rqt_joint_trajectory_controller` or manual commands to find correct joint angles for each feeding phase, then update the `POSES` dict.

### HSV Colour Ranges (vision_node.py)
Save a camera frame, inspect HSV values with Python/OpenCV, update the HSV range table in vision_node.

### EKF Noise (fusion_node.py)
- Process noise `Q`: increase if arm moves fast, decrease for smoother estimates
- Measurement noise `R_base`: lower = trust sensor more

### Camera-to-Plate Alignment
The camera is on `feeder_head`. Joint4 controls the camera pitch. Positive joint4 values tilt the feeder head (and camera) in one direction; test with:
```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 1.5]}" --rate 2
```

---

## Hardware (Real Robot)

- **Servos:** 4x Dynamixel XM430-W350R
- **Camera:** Raspberry Pi Camera V2.1 (or USB cam at /dev/video0)
- **Force Sensor:** HX711 load cell via Arduino
- **Ultrasonic:** HC-SR04
- **Compute:** NVIDIA Jetson Orin (JetPack 6.x, Ubuntu 22.04)
- **Link lengths:** L1=0.042m, L2=0.181m, L3=0.164m, feeder=0.148m

---

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| `Package 'feeding_robot' not found` | Not in workspace src/ | `cp -r feeding_robot ros2_feedbot_ws/src/feeding_robot` |
| Robot not visible in RViz | Wrong fixed frame | Set to `world` |
| Controllers fail to load | `pid_controller` not in Humble | Use `joint_trajectory_controller` with effort+PID |
| Arm doesn't move | Position interface not functional on Fortress | Switch to effort interface with PID gains |
| TF buffer time jumps / RViz blinking | Sim-time jitter on Jetson | Cosmetic; ensure single `robot_state_publisher` instance |
| Food not detected | HSV colours don't match Gazebo rendering | Calibrate HSV ranges; ensure Fortress-compatible materials |
| Camera sees black | Arm collapsed under gravity / wrong pose | Use effort interface with PID; check arm position in Gazebo |
| `Gazebo/Red` not rendering | Classic material syntax doesn't work in Fortress | Use `<ambient>/<diffuse>/<specular>` tags |
| Changes don't take effect | Two install dirs; Gazebo reads from top-level | Edit at `/home/shab/feeding_robot_ws/install/feedbot_description/` |
| Circular dependency error | `feedbot_description` ↔ `feedbot_moveit_config` | Remove `feedbot_moveit_config` from feedbot_description's package.xml |
