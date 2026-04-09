"""
Launch file for the full ARIMA-FFNN feeding system.

Launches all nodes required for the adaptive feeding pipeline:
  1. vision_node        - camera-based multi-fruit detection
  2. force_node         - spoon force sensing
  3. fusion_node        - multi-sensor EKF fusion (camera + sonar)
  4. arima_ffnn         - hybrid ARIMA+FFNN predictive controller
  5. fuzzy_controller   - fuzzy logic force/angle regulation
  6. feeding_fsm        - feeding state machine coordinator
  7. sonar_bridge       - converts ultrasonic Range to distance Float64
  8. mouth_animator     - cycles patient jaw open/close in Gazebo

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
            parameters=[{
                'detection_method': 'hsv',  # 'auto', 'ml', or 'hsv'
            }],
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
        Node(
            package='feedbot_fusion',
            executable='sonar_bridge',
            name='sonar_bridge_node',
            output='screen',
        ),
        Node(
            package='feedbot_fusion',
            executable='mouth_animator',
            name='mouth_animator_node',
            output='screen',
            parameters=[{
                'period': 4.0,
                'max_opening': -0.4,
                'open_threshold': 0.5,
            }],
        ),
        Node(
            package='feedbot_fusion',
            executable='face_node',
            name='face_node',
            output='screen',
            parameters=[{
                'camera_topic': '/feeding_robot/camera/image_raw',
                'mar_threshold': 0.3,
                'process_every_n': 3,
            }],
        ),
    ])
