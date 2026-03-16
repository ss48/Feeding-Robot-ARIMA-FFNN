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
        echo "  Launching in separate terminals..."

        gnome-terminal --title="Vision" -- bash -c \
            "source /opt/ros/humble/setup.bash && \
             source ~/feeding_robot_ws/ros2_feedbot_ws/install/setup.bash && \
             ros2 run feedbot_fusion vision_node; exec bash"

        gnome-terminal --title="Force" -- bash -c \
            "source /opt/ros/humble/setup.bash && \
             source ~/feeding_robot_ws/ros2_feedbot_ws/install/setup.bash && \
             ros2 run feedbot_fusion force_node; exec bash"

        gnome-terminal --title="Fusion" -- bash -c \
            "source /opt/ros/humble/setup.bash && \
             source ~/feeding_robot_ws/ros2_feedbot_ws/install/setup.bash && \
             ros2 run feedbot_fusion fusion_node; exec bash"

        gnome-terminal --title="ARIMA-FFNN" -- bash -c \
            "source /opt/ros/humble/setup.bash && \
             source ~/feeding_robot_ws/ros2_feedbot_ws/install/setup.bash && \
             ros2 run feedbot_fusion arima_ffnn; exec bash"

        gnome-terminal --title="Fuzzy" -- bash -c \
            "source /opt/ros/humble/setup.bash && \
             source ~/feeding_robot_ws/ros2_feedbot_ws/install/setup.bash && \
             ros2 run feedbot_fusion fuzzy_controller; exec bash"

        gnome-terminal --title="FSM" -- bash -c \
            "source /opt/ros/humble/setup.bash && \
             source ~/feeding_robot_ws/ros2_feedbot_ws/install/setup.bash && \
             ros2 run feedbot_fusion feeding_fsm; exec bash"

        echo "All nodes launched in separate terminals."
        ;;

    *)
        echo "Unknown mode: $MODE"
        echo "Usage: $0 [sim|real|predict|fsm|nodes]"
        exit 1
        ;;
esac
