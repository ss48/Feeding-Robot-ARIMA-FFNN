"""
Launch file for real hardware of the Feeding Robot.
Starts ros2_control with Dynamixel hardware interface, robot_state_publisher,
and joint controllers.

Usage:
  ros2 launch feeding_robot hardware.launch.py
  ros2 launch feeding_robot hardware.launch.py port_name:=/dev/ttyACM0
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('feeding_robot')

    # ==================== LAUNCH ARGUMENTS ====================
    port_arg = DeclareLaunchArgument(
        'port_name', default_value='/dev/dynamixel',
        description='Dynamixel USB port (udev symlink or /dev/ttyACMx)'
    )

    use_meshes_arg = DeclareLaunchArgument(
        'use_meshes', default_value='true',
        description='Use STL mesh files'
    )

    camera_device_arg = DeclareLaunchArgument(
        'camera_device', default_value='/dev/video0',
        description='Camera device path'
    )

    teensy_port_arg = DeclareLaunchArgument(
        'teensy_port', default_value='/dev/teensy',
        description='Teensy serial port (force sensor + sonar)'
    )

    # ==================== ROBOT DESCRIPTION ====================
    xacro_file = os.path.join(pkg_dir, 'description', 'feeding_robot.urdf.xacro')
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_sim:=false',
            ' use_meshes:=', LaunchConfiguration('use_meshes'),
        ]),
        value_type=str
    )

    controllers_file = os.path.join(pkg_dir, 'config', 'feeding_robot_controllers.yaml')

    # ==================== ROS2 CONTROL NODE ====================
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file,
        ],
        output='screen',
    )

    # ==================== ROBOT STATE PUBLISHER ====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # ==================== CONTROLLERS ====================
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # ==================== CAMERA (Raspberry Pi Camera) ====================
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='feeding_camera',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
            'camera_frame_id': 'camera_link',
        }],
        remappings=[
            ('/image_raw', '/feeding_robot/camera/image_raw'),
            ('/camera_info', '/feeding_robot/camera/camera_info'),
        ],
        output='screen',
    )

    # ==================== TEENSY BRIDGE (Force Sensor + Sonar) ====================
    teensy_bridge_node = Node(
        package='feedbot_fusion',
        executable='teensy_bridge',
        name='teensy_bridge',
        parameters=[{
            'serial_port': LaunchConfiguration('teensy_port'),
            'baud_rate': 115200,
        }],
        output='screen',
    )

    # ==================== SONAR BRIDGE (routes sonar to plate/mouth topics) ====================
    sonar_bridge_node = Node(
        package='feedbot_fusion',
        executable='sonar_bridge',
        name='sonar_bridge',
        output='screen',
    )

    return LaunchDescription([
        port_arg,
        use_meshes_arg,
        camera_device_arg,
        teensy_port_arg,
        ros2_control_node,
        robot_state_publisher,
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        camera_node,
        teensy_bridge_node,
        sonar_bridge_node,
    ])
