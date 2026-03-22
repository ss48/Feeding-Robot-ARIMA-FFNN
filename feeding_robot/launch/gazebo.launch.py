"""
Launch file for Gazebo simulation of the Feeding Robot.
Delegates to feedbot_description for the robot model (STL meshes,
table, plate, food items, and patient face) while adding the
ROS-Gazebo sensor bridges needed by the feeding system.

Usage:
  ros2 launch feeding_robot gazebo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Use feedbot_description's gazebo launch — it loads the STL-mesh
    # robot with table, plate, food, and patient face built into the URDF,
    # spawns in Gazebo Fortress, sets up PID controllers (joint1-4),
    # bridges sensor topics, and opens RViz.
    feedbot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('feedbot_description'),
                'launch', 'gazebo.launch.py'
            )
        ),
    )

    return LaunchDescription([
        feedbot_desc_launch,
    ])
