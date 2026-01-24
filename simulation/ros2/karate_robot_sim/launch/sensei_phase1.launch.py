from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_ros2_control = LaunchConfiguration("enable_ros2_control")
    package_share = Path(get_package_share_directory("karate_robot_sim"))
    urdf_path = package_share / "urdf" / "sensei_full.urdf.xacro"
    world_path = package_share / "worlds" / "empty.sdf"
    rviz_config = package_share / "rviz" / "sensei.rviz"
    controller_config = package_share / "config" / "ros2_controllers.yaml"
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
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "debug"],
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(enable_ros2_control),
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
            "--ros-args",
            "--log-level",
            "info",
        ],
        condition=IfCondition(enable_ros2_control),
        output="screen",
    )

    left_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "--controller-manager-timeout",
            "60",
            "--ros-args",
            "--log-level",
            "info",
        ],
        condition=IfCondition(enable_ros2_control),
        output="screen",
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
            DeclareLaunchArgument(
                "enable_ros2_control",
                default_value="true",
                description="Enable ros2_control controllers and spawners.",
            ),
            gz_sim,
            robot_state_publisher,
            ros2_control_node,
            TimerAction(period=5.0, actions=[joint_state_broadcaster]),
            TimerAction(period=6.0, actions=[left_arm_controller]),
            spawn_entity,
            rviz,
        ]
    )
