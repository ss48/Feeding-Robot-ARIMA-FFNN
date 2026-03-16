import os
import re
import xacro

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
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

    # Replace package:// URIs with absolute paths so Ignition can find meshes
    robot_description_raw = robot_description_raw.replace(
        "package://feedbot_description/",
        pkg_share + "/"
    )

    robot_description = ParameterValue(
        robot_description_raw,
        value_type=str
    )

    # Write URDF to a temp file for Ignition to read
    urdf_tmp = "/tmp/feedbot_description.urdf"
    with open(urdf_tmp, "w") as f:
        f.write(robot_description_raw)

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
        parameters=[{"robot_description": robot_description}],
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
    # Bridge camera topic from Ignition to ROS 2
    # --------------------------------------------------
    gz_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/camera_sensor/image_raw"],
        output="screen",
    )

    # Controllers are auto-activated by the ign_ros2_control plugin
    # via controllers.yaml — no need to spawn them manually.

    # --------------------------------------------------
    # RViz
    # --------------------------------------------------
    rviz_config = os.path.join(pkg_share, "rviz", "feeding_system.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        ign_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
        rviz,
    ])
