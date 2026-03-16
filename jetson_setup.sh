#!/bin/bash
# =============================================================================
# Jetson Orin Deployment Script for Feeding Robot ARIMA-FFNN
#
# Prerequisites:
#   - JetPack 6.x installed (Ubuntu 22.04 based)
#   - Internet connection
#
# Usage:
#   chmod +x jetson_setup.sh
#   ./jetson_setup.sh
# =============================================================================

set -e  # Exit on any error

echo "============================================"
echo "  Feeding Robot - Jetson Orin Setup"
echo "============================================"

# ---------------------------------------------------------------------------
# 1. System update
# ---------------------------------------------------------------------------
echo ""
echo "[1/8] Updating system packages..."
sudo apt update && sudo apt upgrade -y

# ---------------------------------------------------------------------------
# 2. Install ROS 2 Humble (if not already installed)
# ---------------------------------------------------------------------------
if ! command -v ros2 &> /dev/null; then
    echo ""
    echo "[2/8] Installing ROS 2 Humble..."

    # Set locale
    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # Add ROS 2 repo
    sudo apt install -y software-properties-common curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-humble-ros-base ros-dev-tools
else
    echo ""
    echo "[2/8] ROS 2 already installed, skipping..."
fi

# ---------------------------------------------------------------------------
# 3. Install ROS 2 packages needed by the project
# ---------------------------------------------------------------------------
echo ""
echo "[3/8] Installing ROS 2 dependencies..."
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-control-msgs \
    ros-humble-std-msgs \
    ros-humble-tf2-ros \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    python3-colcon-common-extensions \
    python3-rosdep

# ---------------------------------------------------------------------------
# 4. Install Python dependencies
# ---------------------------------------------------------------------------
echo ""
echo "[4/8] Installing Python dependencies..."
pip3 install --upgrade pip
pip3 install numpy opencv-python trimesh

# ---------------------------------------------------------------------------
# 5. Clone the repository
# ---------------------------------------------------------------------------
REPO_DIR="$HOME/feeding_robot_ws"

if [ -d "$REPO_DIR" ]; then
    echo ""
    echo "[5/8] Repository already exists at $REPO_DIR, pulling latest..."
    cd "$REPO_DIR"
    git pull origin main
else
    echo ""
    echo "[5/8] Cloning repository..."
    git clone git@github.com:ss48/Feeding-Robot-ARIMA-FFNN.git "$REPO_DIR"
    cd "$REPO_DIR"
fi

# ---------------------------------------------------------------------------
# 6. Initialise rosdep and install ROS dependencies
# ---------------------------------------------------------------------------
echo ""
echo "[6/8] Running rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
fi
rosdep update
cd "$REPO_DIR/ros2_feedbot_ws"
rosdep install --from-paths src --ignore-src -r -y || true

# ---------------------------------------------------------------------------
# 7. Build the workspace
# ---------------------------------------------------------------------------
echo ""
echo "[7/8] Building ROS 2 workspace..."
source /opt/ros/humble/setup.bash
cd "$REPO_DIR/ros2_feedbot_ws"

colcon build --symlink-install --parallel-workers $(nproc)

# ---------------------------------------------------------------------------
# 8. Configure environment auto-sourcing
# ---------------------------------------------------------------------------
echo ""
echo "[8/8] Configuring shell environment..."

BASHRC_MARKER="# >>> feeding_robot >>>"
if ! grep -q "$BASHRC_MARKER" ~/.bashrc; then
    cat >> ~/.bashrc << 'BASHEOF'

# >>> feeding_robot >>>
source /opt/ros/humble/setup.bash
source ~/feeding_robot_ws/ros2_feedbot_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# <<< feeding_robot <<<
BASHEOF
    echo "Added ROS 2 sourcing to ~/.bashrc"
else
    echo "Shell environment already configured"
fi

echo ""
echo "============================================"
echo "  Setup complete!"
echo ""
echo "  To start using:"
echo "    source ~/.bashrc"
echo ""
echo "  To launch the feeding system:"
echo "    ros2 launch feedbot_fusion feeding_system.launch.py"
echo "============================================"
