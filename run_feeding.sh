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
        echo "Step 1: Launching Gazebo with robot..."
        ros2 launch feedbot_description gazebo.launch.py &
        GAZEBO_PID=$!
        echo "  Gazebo PID: $GAZEBO_PID"

        # Wait for Gazebo and controllers to initialise
        echo "  Waiting 10s for Gazebo to start..."
        sleep 10

        echo ""
        echo "Step 2: Launching feeding system nodes..."
        ros2 launch feedbot_fusion feeding_system.launch.py

        # Cleanup
        kill $GAZEBO_PID 2>/dev/null || true
        ;;

    real)
        echo "Starting feeding system on real hardware..."
        echo ""
        echo "Make sure:"
        echo "  - Dynamixel servos are powered and connected"
        echo "  - Camera is connected (/dev/video0)"
        echo "  - Force sensor (HX711) is connected via Arduino"
        echo "  - Joint controllers are already running"
        echo ""
        read -p "Press Enter to continue..."

        ros2 launch feedbot_fusion feeding_system.launch.py
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

        NODES=(vision_node force_node fusion_node arima_ffnn fuzzy_controller feeding_fsm)
        PIDS=()

        for NODE in "${NODES[@]}"; do
            echo "  Starting $NODE (log: $LOG_DIR/$NODE.log)"
            ros2 run feedbot_fusion "$NODE" > "$LOG_DIR/$NODE.log" 2>&1 &
            PIDS+=($!)
        done

        echo ""
        echo "All nodes launched. PIDs: ${PIDS[*]}"
        echo "Logs: $LOG_DIR/"
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
