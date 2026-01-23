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
    urdf_path = package_share / "urdf" / "sensei_full.urdf.xacro"
    world_path = package_share / "worlds" / "empty.sdf"
    rviz_config = package_share / "rviz" / "sensei.rviz"
    robot_description = xacro.process_file(str(urdf_path)).toxml()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("ros_gz_sim"),
                "/launch/gz_sim.launch.py",
            ]
        ),
        launch_arguments={"gz_args": f"-r -s {world_path}"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", str(rviz_config)],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            "sensei_world",
            "-name",
            "sensei_full",
            "-string",
            robot_description,
            "-allow_renaming",
            "true",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_sim,
            robot_state_publisher,
            joint_state_publisher,
            spawn_entity,
            rviz,
        ]
    )
