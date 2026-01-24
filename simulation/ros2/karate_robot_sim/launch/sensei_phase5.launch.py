from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    enable_ros2_control = LaunchConfiguration("enable_ros2_control")
    phase1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("karate_robot_sim"),
                "/launch/sensei_phase1.launch.py",
            ]
        ),
        launch_arguments={"enable_ros2_control": enable_ros2_control}.items(),
    )

    register_bridge = Node(
        package="karate_robot_bridge",
        executable="register_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    reflex_guard = Node(
        package="karate_robot_bridge",
        executable="reflex_guard",
        output="screen",
        parameters=[{"spike_after_sec": 12.0}],
    )

    kick_sequence = Node(
        package="karate_robot_bridge",
        executable="kick_sequence",
        output="screen",
    )

    training_env = Node(
        package="karate_robot_bridge",
        executable="training_env",
        output="screen",
        parameters=[{"episode_timeout_sec": 20.0}],
    )

    desired_pose_joint_state = Node(
        package="karate_robot_bridge",
        executable="desired_pose_joint_state",
        output="screen",
    )

    right_leg_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_leg_controller",
            "--controller-manager-timeout",
            "60",
            "--ros-args",
            "--log-level",
            "info",
        ],
        condition=IfCondition(enable_ros2_control),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_ros2_control",
                default_value="false",
                description="Disable ros2_control by default on macOS.",
            ),
            phase1,
            TimerAction(
                period=7.0,
                actions=[right_leg_controller],
                condition=IfCondition(enable_ros2_control),
            ),
            register_bridge,
            reflex_guard,
            kick_sequence,
            desired_pose_joint_state,
            training_env,
        ]
    )
