"""
Launch file for Gazebo simulation of the Feeding Robot.
Spawns the robot in Gazebo with ros2_control controllers, bridges
camera / ultrasonic / jaw topics, and launches the feeding system.

Usage:
  ros2 launch feeding_robot gazebo.launch.py
  ros2 launch feeding_robot gazebo.launch.py world:=feeding_table.sdf
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('feeding_robot')

    # ==================== LAUNCH ARGUMENTS ====================
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'feeding_table.sdf'),
        description='Path to the Gazebo world file'
    )

    use_meshes_arg = DeclareLaunchArgument(
        'use_meshes', default_value='false',
        description='Use STL mesh files instead of primitive shapes'
    )

    # ==================== ROBOT DESCRIPTION ====================
    xacro_file = os.path.join(pkg_dir, 'description', 'feeding_robot.urdf.xacro')
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_sim:=true',
            ' use_meshes:=', LaunchConfiguration('use_meshes')
        ]),
        value_type=str
    )

    # ==================== ROBOT STATE PUBLISHER ====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen'
    )

    # ==================== GAZEBO ====================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': ['-r -v 4 ', LaunchConfiguration('world')],
        }.items(),
    )

    # Spawn the robot entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'feeding_robot',
            '-z', '0.0',
        ],
        output='screen'
    )

    # ==================== ROS-GAZEBO BRIDGE ====================
    # Bridge clock, camera image, ultrasonic range, and jaw command
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Camera image (Gazebo -> ROS)
            '/feeding_robot/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # Camera info (Gazebo -> ROS)
            '/feeding_robot/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Ultrasonic range (Gazebo -> ROS)
            '/feeding_robot/ultrasonic/range@sensor_msgs/msg/Range[gz.msgs.LaserScan',
            # Patient jaw command (ROS -> Gazebo)
            '/patient_head/jaw_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen'
    )

    # ==================== CONTROLLERS ====================
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # Delay controller spawning to ensure Gazebo is ready
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

    # ==================== RVIZ2 ====================
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_config.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        use_meshes_arg,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        rviz2,
    ])
