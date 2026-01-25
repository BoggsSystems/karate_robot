from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("karate_robot_sim"))
    rviz_config = package_share / "rviz" / "sensei.rviz"
    urdf_path = package_share / "urdf" / "sensei_full.urdf.xacro"
    robot_description = xacro.process_file(str(urdf_path)).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", str(rviz_config)],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            joint_state_publisher,
            rviz,
        ]
    )
