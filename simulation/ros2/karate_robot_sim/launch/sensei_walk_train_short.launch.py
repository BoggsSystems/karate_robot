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

    desired_pose_joint_state = Node(
        package="karate_robot_bridge",
        executable="desired_pose_joint_state",
        output="screen",
        parameters=[{"use_sim_time": False, "publish_rate_hz": 30.0}],
    )

    contact_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/model/sensei_full/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/world/sensei_world/model/sensei_full/link/l_foot_link/sensor/left_foot_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
            "/world/sensei_world/model/sensei_full/link/r_foot_link/sensor/right_foot_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
        ],
        remappings=[
            ("/model/sensei_full/odometry", "odom"),
            (
                "/world/sensei_world/model/sensei_full/link/l_foot_link/sensor/left_foot_contact/contact",
                "contact/left_foot_raw",
            ),
            (
                "/world/sensei_world/model/sensei_full/link/r_foot_link/sensor/right_foot_contact/contact",
                "contact/right_foot_raw",
            ),
        ],
    )

    left_contact_bool = Node(
        package="karate_robot_bridge",
        executable="contact_bool",
        output="screen",
        parameters=[
            {"contact_topic": "contact/left_foot_raw"},
            {"bool_topic": "contact/left_foot"},
            {"publish_rate_hz": 20.0},
        ],
    )

    right_contact_bool = Node(
        package="karate_robot_bridge",
        executable="contact_bool",
        output="screen",
        parameters=[
            {"contact_topic": "contact/right_foot_raw"},
            {"bool_topic": "contact/right_foot"},
            {"publish_rate_hz": 20.0},
        ],
    )

    sensor_watchdog = Node(
        package="karate_robot_bridge",
        executable="sensor_watchdog",
        output="screen",
        parameters=[
            {"odom_topic": "odom"},
            {"left_contact_topic": "contact/left_foot"},
            {"right_contact_topic": "contact/right_foot"},
            {"timeout_sec": 1.0},
            {"report_period_sec": 2.0},
        ],
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
            {"reward_profile": "stable"},
            {"curriculum_enabled": False},
            {"episode_timeout_sec": 12.0},
            {"use_sim_time": False},
        ],
    )

    rl_trainer = Node(
        package="karate_robot_bridge",
        executable="rl_trainer",
        output="screen",
        parameters=[
            {"algo": "PPO"},
            {"total_timesteps": 2000},
            {"log_dir": "/tmp/karate_robot_walk_short"},
            {"model_path": "/tmp/karate_robot_walk_short_policy"},
        ],
    )

    return LaunchDescription(
        [
            gz_sim,
            robot_state_publisher,
            spawn_entity,
            desired_pose_joint_state,
            contact_bridge,
            left_contact_bool,
            right_contact_bool,
            sensor_watchdog,
            training_env,
            rl_trainer,
        ]
    )
