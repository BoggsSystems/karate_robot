from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("karate_robot_sim"),
                "/launch/sensei_full_body.launch.py",
            ]
        )
    )

    register_bridge = Node(
        package="karate_robot_bridge",
        executable="register_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    test_pose_publisher = Node(
        package="karate_robot_bridge",
        executable="test_pose_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            gazebo_launch,
            register_bridge,
            test_pose_publisher,
        ]
    )
