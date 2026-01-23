from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = Path(get_package_share_directory("karate_robot_sim"))
    rviz_config = package_share / "rviz" / "sensei.rviz"
    urdf_path = package_share / "urdf" / "sensei_arm.urdf.xacro"
    robot_description = xacro.process_file(str(urdf_path)).toxml()

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("karate_robot_sim"),
                "/launch/sensei_full.launch.py",
            ]
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", str(rviz_config)],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    return LaunchDescription([sim_launch, joint_state_publisher, rviz])
