from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    phase1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("karate_robot_sim"),
                "/launch/sensei_phase1.launch.py",
            ]
        )
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

    strategist_node = Node(
        package="karate_robot_bridge",
        executable="strategist_node",
        output="screen",
        parameters=[{"identity_override": True}],
    )

    return LaunchDescription(
        [
            phase1,
            register_bridge,
            reflex_guard,
            strategist_node,
        ]
    )
