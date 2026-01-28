"""
Launch file for evaluating a trained walking policy.
Runs the policy in simulation and collects metrics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/tmp/karate_robot_walk/walking_policy_final.zip",
        description="Path to trained policy model",
    )
    
    episodes_arg = DeclareLaunchArgument(
        "episodes",
        default_value="20",
        description="Number of evaluation episodes",
    )
    
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run without visualization",
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": open(
                "/ros2_ws/src/karate_robot/simulation/ros2/karate_robot_sim/urdf/sensei_full.urdf.xacro"
            ).read() if False else ""  # Loaded via xacro in actual use
        }],
    )

    # Training environment (in eval mode)
    training_env = Node(
        package="karate_robot_bridge",
        executable="training_env",
        output="screen",
        parameters=[
            {"reward_mode": "walk"},
            {"domain_randomization": "off"},  # No randomization during eval
            {"episode_timeout_sec": 30.0},
        ],
    )

    # Policy runner (inference mode)
    policy_runner = Node(
        package="karate_robot_bridge",
        executable="policy_runner",
        output="screen",
        parameters=[
            {"model_path": LaunchConfiguration("model_path")},
            {"eval_episodes": LaunchConfiguration("episodes")},
        ],
    )

    return LaunchDescription([
        model_path_arg,
        episodes_arg,
        headless_arg,
        robot_state_publisher,
        training_env,
        policy_runner,
    ])
