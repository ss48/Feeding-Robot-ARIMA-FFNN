"""
Launch file for the full ARIMA-FFNN feeding system.

Launches all nodes required for the adaptive feeding pipeline:
  1. vision_node        – camera-based food detection
  2. force_node         – spoon force sensing
  3. fusion_node        – multi-sensor fusion + distance estimation
  4. arima_ffnn         – hybrid ARIMA+FFNN predictive controller
  5. fuzzy_controller   – fuzzy logic force/angle regulation
  6. feeding_fsm        – feeding state machine coordinator

Usage:
  ros2 launch feedbot_fusion feeding_system.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='feedbot_fusion',
            executable='vision_node',
            name='vision_node',
            output='screen',
        ),
        Node(
            package='feedbot_fusion',
            executable='force_node',
            name='force_node',
            output='screen',
        ),
        Node(
            package='feedbot_fusion',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
        ),
        Node(
            package='feedbot_fusion',
            executable='arima_ffnn',
            name='arima_ffnn_node',
            output='screen',
        ),
        Node(
            package='feedbot_fusion',
            executable='fuzzy_controller',
            name='fuzzy_controller_node',
            output='screen',
        ),
        Node(
            package='feedbot_fusion',
            executable='feeding_fsm',
            name='feeding_fsm_node',
            output='screen',
        ),
    ])
