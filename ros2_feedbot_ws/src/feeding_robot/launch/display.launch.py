"""
Launch file for RViz2 visualization of the Feeding Robot.
Displays the URDF with interactive joint sliders (no physics simulation).

Usage:
  ros2 launch feeding_robot display.launch.py
  ros2 launch feeding_robot display.launch.py use_meshes:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('feeding_robot')

    # Launch arguments
    use_meshes_arg = DeclareLaunchArgument(
        'use_meshes', default_value='false',
        description='Use STL mesh files instead of primitive shapes'
    )

    # Process xacro to generate URDF
    xacro_file = os.path.join(pkg_dir, 'description', 'feeding_robot.urdf.xacro')
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_sim:=false',
            ' use_meshes:=', LaunchConfiguration('use_meshes')
        ]),
        value_type=str
    )

    # Robot State Publisher - publishes TF transforms from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        output='screen'
    )

    # Joint State Publisher GUI - interactive sliders for each joint
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_config.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        use_meshes_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
