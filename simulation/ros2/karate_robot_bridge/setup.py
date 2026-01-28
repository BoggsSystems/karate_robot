from setuptools import setup

package_name = "karate_robot_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            "share/" + package_name + "/config",
            ["config/training_schema.yaml", "config/walk_reward_profiles.yaml"],
        ),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="karate_robot",
    maintainer_email="dev@karate-robot.local",
    description="Bridge between strategist intent and simulated actuation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "register_bridge = karate_robot_bridge.register_bridge:main",
            "reflex_guard = karate_robot_bridge.reflex_guard:main",
            "strategist_node = karate_robot_bridge.strategist_node:main",
            "training_env = karate_robot_bridge.training_env_node:main",
            "desired_pose_joint_state = karate_robot_bridge.desired_pose_joint_state:main",
            "training_agent = karate_robot_bridge.training_agent_node:main",
            "rl_trainer = karate_robot_bridge.rl_trainer_node:main",
            "policy_runner = karate_robot_bridge.policy_runner_node:main",
            "start_pose_publisher = karate_robot_bridge.start_pose_publisher:main",
            "kick_loop = karate_robot_bridge.kick_loop_node:main",
            "contact_bool = karate_robot_bridge.contact_bool_node:main",
            "sensor_watchdog = karate_robot_bridge.sensor_watchdog_node:main",
        ],
    },
)
