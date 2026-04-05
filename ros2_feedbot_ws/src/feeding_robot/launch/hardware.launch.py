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
        'port_name', default_value='/dev/ttyACM0',
        description='Dynamixel USB port'
    )

    use_meshes_arg = DeclareLaunchArgument(
        'use_meshes', default_value='true',
        description='Use STL mesh files'
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

    return LaunchDescription([
        port_arg,
        use_meshes_arg,
        ros2_control_node,
        robot_state_publisher,
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
    ])
