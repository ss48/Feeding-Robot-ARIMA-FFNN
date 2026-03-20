import os
import re
import xacro

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory("feedbot_description")
    xacro_file = os.path.join(pkg_share, "urdf", "my_robot.urdf.xacro")

    # --------------------------------------------------
    # Process Xacro
    # --------------------------------------------------
    doc = xacro.process_file(xacro_file)
    robot_description_raw = doc.toxml()

    # Remove XML comments (prevents ROS2 CLI truncation bug)
    robot_description_raw = re.sub(
        r'<!--.*?-->',
        '',
        robot_description_raw,
        flags=re.DOTALL
    )

    # RViz version: keep package:// URIs so RViz can resolve meshes
    robot_description_rviz = ParameterValue(
        robot_description_raw,
        value_type=str
    )

    # Ignition version: replace package:// with absolute paths
    robot_description_ign = robot_description_raw.replace(
        "package://feedbot_description/",
        pkg_share + "/"
    )

    # Write Ignition URDF to a temp file for spawning
    urdf_tmp = "/tmp/feedbot_description.urdf"
    with open(urdf_tmp, "w") as f:
        f.write(robot_description_ign)

    # --------------------------------------------------
    # Set resource path so Ignition can find meshes
    # --------------------------------------------------
    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=os.path.dirname(pkg_share),
    )

    # --------------------------------------------------
    # Ignition Gazebo (Fortress, shipped with Humble)
    # --------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # --------------------------------------------------
    # Robot State Publisher
    # --------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description_rviz,
            "use_sim_time": True,
        }],
        output="screen",
    )

    # --------------------------------------------------
    # Spawn Robot
    # --------------------------------------------------
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "feedbot",
            "-file", urdf_tmp,
        ],
        output="screen",
    )

    # --------------------------------------------------
    # Bridge sensor topics from Ignition to ROS 2
    # --------------------------------------------------
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Clock (required for use_sim_time)
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Camera image (Gz -> ROS)
            "/feeding_robot/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            # Ultrasonic lidar scan (Gz -> ROS)
            "/feeding_robot/ultrasonic/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # Force/torque sensor on spoon joint (Gz -> ROS)
            "/spoon/wrench@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench",
        ],
        output="screen",
    )

    # --------------------------------------------------
    # Controllers — spawn joint_state_broadcaster so TF
    # frames are published for all revolute joints, then
    # spawn each PID joint controller after it exits.
    # --------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    joint1_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint1_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    joint2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint2_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    joint3_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint3_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    joint4_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint4_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # Delay spawning until Gazebo + controller_manager are ready
    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner],
    )

    # Spawn joint controllers after joint_state_broadcaster is up
    delayed_joint_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                joint1_controller_spawner,
                joint2_controller_spawner,
                joint3_controller_spawner,
                joint4_controller_spawner,
            ],
        )
    )

    # --------------------------------------------------
    # RViz
    # --------------------------------------------------
    rviz_config = os.path.join(pkg_share, "rviz", "feeding_system.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        ign_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
        delayed_joint_state_broadcaster,
        delayed_joint_controllers,
        rviz,
    ])
