from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Bushido note: simulation is the dojo where truth meets intention.
    package_share = Path(get_package_share_directory("karate_robot_sim"))
    urdf_path = package_share / "urdf" / "sensei_arm.urdf.xacro"
    robot_description = xacro.process_file(str(urdf_path)).toxml()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
