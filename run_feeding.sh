#!/bin/bash
# =============================================================================
# Quick-start script for running the Feeding Robot on Jetson Orin
#
# Usage:
#   ./run_feeding.sh [mode]
#
# Modes:
#   sim       - Launch Gazebo simulation + feeding system (default)
#   real      - Launch feeding system only (real hardware)
#   predict   - Launch only the ARIMA-FFNN predictor (for testing)
#   fsm       - Launch only the feeding FSM (for testing)
#   nodes     - Launch all nodes individually (for debugging)
#
# Sensor Fusion Pipeline:
#   Camera (vision_node)  ---bearing+depth--> |                  |
#   Ultrasonic (sonar_bridge) ---range------> | fusion_node (EKF)| --> food_position_3d
#   Force sensor (force_node) ---force------> |  6-state KF      | --> plate_distance
#   Joint states ----------joints-----------> |  Kalman Filter   | --> mouth_distance
#   ARIMA-FFNN (arima_ffnn) ---prediction---> |                  | --> fusion_confidence
#
# Food Localisation Flow:
#   1. Camera identifies food via ML (jetson-inference detectNet) or HSV fallback
#   2. Pinhole camera model computes bearing angles + monocular depth
#   3. Ultrasonic sensor measures range to food on plate
#   4. EKF fuses camera bearing + ultrasonic range into 3D food position
#   5. FSM uses IK from fused 3D position to compute precise fork angles
#   6. Fuzzy controller regulates force/angle based on food type
#
# Manual Arm Control (PID controllers):
#   ros2 topic pub /joint1_controller/reference \
#     control_msgs/msg/MultiDOFCommand \
#     "{dof_names: ['joint1'], values: [1.0]}" --once
#
#   ros2 topic pub /joint2_controller/reference \
#     control_msgs/msg/MultiDOFCommand \
#     "{dof_names: ['joint2'], values: [0.5]}" --once
#
# Monitor Sensor Fusion:
#   ros2 topic echo /food_position_3d     # fused 3D food position (m)
#   ros2 topic echo /plate_distance       # EKF plate distance (cm)
#   ros2 topic echo /mouth_distance       # EKF mouth distance (cm)
#   ros2 topic echo /fusion_confidence    # overall confidence [0-1]
#   ros2 topic echo /sensor_health        # per-sensor health scores
#   ros2 topic echo /food_bearing         # camera bearing + depth
#   ros2 topic echo /sonar_plate_distance # ultrasonic range (cm)
#   ros2 topic echo /feeding_state        # current FSM state
# =============================================================================

set -e

# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/feeding_robot_ws/ros2_feedbot_ws/install/setup.bash

MODE="${1:-sim}"

echo "============================================"
echo "  Feeding Robot - Mode: $MODE"
echo "============================================"

case "$MODE" in

    sim)
        echo "Starting Gazebo simulation + feeding system..."
        echo ""
        echo "Sensor Fusion: Camera + Ultrasonic + Force + Joints"
        echo "  EKF Kalman Filter fuses all sensors for 3D food localisation"
        echo ""
        # Log directory for node output
        LOG_DIR="/tmp/feedbot_logs"
        mkdir -p "$LOG_DIR"

        echo "Step 1: Launching Gazebo with robot..."
        ros2 launch feedbot_description gazebo.launch.py \
            > "$LOG_DIR/gazebo.log" 2>&1 &
        GAZEBO_PID=$!
        echo "  Gazebo PID: $GAZEBO_PID"
        echo "  Log: $LOG_DIR/gazebo.log"

        # Wait for Gazebo and controllers to initialise
        echo "  Waiting 12s for Gazebo to start..."
        sleep 12

        echo ""
        echo "Step 2: Launching feeding system nodes..."
        echo "  - vision_node:       ML food detection (jetson-inference) or HSV fallback"
        echo "  - force_node:        Force sensor filtering"
        echo "  - sonar_bridge:      Ultrasonic range conversion"
        echo "  - fusion_node:       EKF Kalman Filter (camera+ultrasonic+force+joints)"
        echo "  - arima_ffnn:        ARIMA-FFNN predictive controller"
        echo "  - fuzzy_controller:  Fuzzy logic force/angle regulation"
        echo "  - feeding_fsm:       State machine + IK food pickup"
        echo "  - mouth_animator:    Patient jaw animation (sim only)"
        echo ""
        ros2 launch feedbot_fusion feeding_system.launch.py \
            > "$LOG_DIR/feeding_system.log" 2>&1 &
        FEEDING_PID=$!
        echo "  Feeding system PID: $FEEDING_PID"
        echo "  Log: $LOG_DIR/feeding_system.log"

        sleep 3
        echo ""
        echo "============================================"
        echo "  System ready!"
        echo "  Press SPACE to start feeding cycle."
        echo "  Press Q to quit."
        echo "  Logs: $LOG_DIR/"
        echo "============================================"
        echo ""

        # Cleanup handler
        cleanup() {
            echo ""
            echo "Shutting down..."
            kill $FEEDING_PID 2>/dev/null || true
            kill $GAZEBO_PID 2>/dev/null || true
            exit 0
        }
        trap cleanup INT TERM

        # Listen for spacebar — read from /dev/tty to avoid interference
        # from backgrounded process output
        while true; do
            if read -rsn1 key < /dev/tty 2>/dev/null; then
                if [ "$key" = " " ]; then
                    echo "[SPACE] Starting feeding cycle..."
                    ros2 topic pub /feeding_start std_msgs/msg/Bool "{data: true}" --once \
                        > /dev/null 2>&1 &
                elif [ "$key" = "q" ] || [ "$key" = "Q" ]; then
                    cleanup
                fi
            fi
        done
        ;;

    real)
        echo "Starting feeding system on real hardware..."
        echo ""
        echo "Make sure:"
        echo "  - Dynamixel servos are powered and connected"
        echo "  - Camera is connected (/dev/video0)"
        echo "  - Force sensor (HX711) is connected via Arduino"
        echo "  - Ultrasonic sensor (HC-SR04) is connected"
        echo "  - Joint controllers are already running"
        echo ""
        echo "Sensor Fusion Pipeline:"
        echo "  Camera -> bearing angles -> EKF"
        echo "  HC-SR04 -> range (cm)    -> EKF  => 3D food position"
        echo "  HX711   -> force (N)     -> EKF"
        echo "  Joints  -> angles (rad)  -> EKF"
        echo ""
        read -p "Press Enter to continue..."

        LOG_DIR="/tmp/feedbot_logs"
        mkdir -p "$LOG_DIR"

        ros2 launch feedbot_fusion feeding_system.launch.py \
            > "$LOG_DIR/feeding_system.log" 2>&1 &
        FEEDING_PID=$!

        cleanup() {
            echo ""
            echo "Shutting down..."
            kill $FEEDING_PID 2>/dev/null || true
            exit 0
        }
        trap cleanup INT TERM

        sleep 3
        echo ""
        echo "============================================"
        echo "  System ready! Press SPACE to start feeding."
        echo "  Press Q to quit."
        echo "  Log: $LOG_DIR/feeding_system.log"
        echo "============================================"
        echo ""

        while true; do
            if read -rsn1 key < /dev/tty 2>/dev/null; then
                if [ "$key" = " " ]; then
                    echo "[SPACE] Starting feeding cycle..."
                    ros2 topic pub /feeding_start std_msgs/msg/Bool "{data: true}" --once \
                        > /dev/null 2>&1 &
                elif [ "$key" = "q" ] || [ "$key" = "Q" ]; then
                    cleanup
                fi
            fi
        done
        ;;

    predict)
        echo "Starting ARIMA-FFNN predictor only..."
        ros2 run feedbot_fusion arima_ffnn
        ;;

    fsm)
        echo "Starting feeding FSM only..."
        ros2 run feedbot_fusion feeding_fsm
        ;;

    nodes)
        echo "Starting individual nodes (for debugging)..."
        LOG_DIR="/tmp/feedbot_logs"
        mkdir -p "$LOG_DIR"

        NODES=(vision_node force_node sonar_bridge fusion_node arima_ffnn fuzzy_controller feeding_fsm)
        PIDS=()

        for NODE in "${NODES[@]}"; do
            echo "  Starting $NODE (log: $LOG_DIR/$NODE.log)"
            ros2 run feedbot_fusion "$NODE" > "$LOG_DIR/$NODE.log" 2>&1 &
            PIDS+=($!)
        done

        echo ""
        echo "All nodes launched. PIDs: ${PIDS[*]}"
        echo "Logs: $LOG_DIR/"
        echo ""
        echo "Monitor sensor fusion:"
        echo "  ros2 topic echo /food_position_3d"
        echo "  ros2 topic echo /fusion_confidence"
        echo "  ros2 topic echo /feeding_state"
        echo ""
        echo "Press Ctrl+C to stop all nodes."

        trap 'echo "Stopping all nodes..."; kill "${PIDS[@]}" 2>/dev/null; exit 0' INT TERM
        wait
        ;;

    *)
        echo "Unknown mode: $MODE"
        echo "Usage: $0 [sim|real|predict|fsm|nodes]"
        exit 1
        ;;
esac
