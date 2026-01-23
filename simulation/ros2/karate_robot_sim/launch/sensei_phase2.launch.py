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

    test_left_arm_publisher = Node(
        package="karate_robot_sim",
        executable="test_left_arm_publisher",
        output="screen",
    )

    reflex_guard = Node(
        package="karate_robot_bridge",
        executable="reflex_guard",
        output="screen",
        parameters=[{"spike_after_sec": 12.0}],
    )

    return LaunchDescription(
        [
            phase1,
            test_left_arm_publisher,
            reflex_guard,
        ]
    )
