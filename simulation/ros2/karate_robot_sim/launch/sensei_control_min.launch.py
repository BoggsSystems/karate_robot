from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("karate_robot_sim"))
    urdf_path = package_share / "urdf" / "sensei_full.urdf.xacro"
    controller_config = package_share / "config" / "ros2_controllers.yaml"
    robot_description = xacro.process_file(str(urdf_path)).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "debug"],
        parameters=[
            {"robot_description": robot_description, "use_sim_time": False},
            controller_config,
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "60",
            "--inactive",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster,
        ]
    )
