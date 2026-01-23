from setuptools import setup

package_name = "karate_robot_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="karate_robot",
    maintainer_email="dev@karate-robot.local",
    description="Bridge between strategist intent and simulated actuation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "register_bridge = karate_robot_bridge.register_bridge:main",
            "test_pose_publisher = karate_robot_bridge.test_pose_publisher:main",
            "reflex_guard = karate_robot_bridge.reflex_guard:main",
            "strategist_node = karate_robot_bridge.strategist_node:main",
            "training_env = karate_robot_bridge.training_env_node:main",
        ],
    },
)
