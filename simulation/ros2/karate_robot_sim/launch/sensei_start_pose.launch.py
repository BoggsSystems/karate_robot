from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("karate_robot_sim"))
    urdf_path = package_share / "urdf" / "sensei_full.urdf.xacro"
    robot_description = xacro.process_file(str(urdf_path)).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    desired_pose_joint_state = Node(
        package="karate_robot_bridge",
        executable="desired_pose_joint_state",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    start_pose_publisher = Node(
        package="karate_robot_bridge",
        executable="start_pose_publisher",
        output="screen",
        parameters=[
            {"pose_mode": "kamae", "publish_delay_sec": 0.2, "use_sim_time": False}
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            str(package_share / "rviz" / "sensei.rviz"),
        ],
        parameters=[{"use_sim_time": False}],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            desired_pose_joint_state,
            start_pose_publisher,
            rviz,
        ]
    )
