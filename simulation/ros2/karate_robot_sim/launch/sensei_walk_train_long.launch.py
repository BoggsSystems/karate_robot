from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("karate_robot_sim"))
    urdf_path = package_share / "urdf" / "sensei_full.urdf.xacro"
    robot_description = xacro.process_file(str(urdf_path)).toxml()

    # Launch arguments
    timesteps_arg = DeclareLaunchArgument(
        "timesteps",
        default_value="10000000",
        description="Total training timesteps",
    )
    resume_arg = DeclareLaunchArgument(
        "resume",
        default_value="",
        description="Path to checkpoint to resume from",
    )

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

    training_env = Node(
        package="karate_robot_bridge",
        executable="training_env",
        output="screen",
        parameters=[
            {"action_smoothing_alpha": 0.5},
            {"action_position_min": -1.2},
            {"action_position_max": 1.2},
            {"action_max_delta_per_sec": 2.0},
            {"reward_mode": "walk"},
            {"reward_profile": "baseline"},
            {"curriculum_enabled": True},
            {"curriculum_stage_episodes": 50},
            {"curriculum_profiles": ["stable", "baseline", "forward"]},
            {"episode_timeout_sec": 20.0},
            {"walk_frequency_hz": 1.2},
            {"walk_hip_swing": 0.5},
            {"walk_knee_lift": 0.6},
            {"walk_ankle_lift": 0.25},
            {"walk_ramp_sec": 5.0},
            {"walk_min_scale": 0.3},
            {"walk_forward_velocity_target": 0.3},
            {"walk_forward_velocity_weight": 0.6},
            {"walk_upright_weight": 0.2},
            {"walk_fall_pitch_rad": 0.9},
            {"walk_fall_roll_rad": 0.9},
            {"walk_min_base_height": 0.0},
            {"walk_contact_weight": 0.2},
            {"walk_air_timeout_sec": 0.6},
            {"use_sim_time": False},
        ],
    )

    rl_trainer = Node(
        package="karate_robot_bridge",
        executable="rl_trainer",
        output="screen",
        parameters=[
            {"algo": "PPO"},
            {"total_timesteps": LaunchConfiguration("timesteps")},
            {"log_dir": "/tmp/karate_robot_walk"},
            {"model_path": "/tmp/karate_robot_walk_policy"},
            {"checkpoint_freq": 100000},
            {"resume_path": LaunchConfiguration("resume")},
        ],
    )

    return LaunchDescription(
        [
            timesteps_arg,
            resume_arg,
            robot_state_publisher,
            desired_pose_joint_state,
            training_env,
            rl_trainer,
        ]
    )
