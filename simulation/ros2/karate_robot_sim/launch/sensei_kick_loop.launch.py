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
        parameters=[{"use_sim_time": False, "publish_rate_hz": 30.0}],
    )

    kick_loop = Node(
        package="karate_robot_bridge",
        executable="kick_loop",
        output="screen",
        parameters=[
            {"kick_period_sec": 3.0},
            {"chamber_sec": 0.5},
            {"extend_sec": 0.3},
            {"retract_sec": 0.2},
            {"kick_side": "left"},
            {"use_sim_time": False},
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
            kick_loop,
            rviz,
        ]
    )
