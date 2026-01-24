from setuptools import setup

package_name = "karate_robot_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/sensei_sim.launch.py",
                "launch/sensei_gazebo.launch.py",
                "launch/sensei_full.launch.py",
                "launch/sensei_full_body.launch.py",
                "launch/sensei_phase0.launch.py",
                "launch/sensei_phase1.launch.py",
                "launch/sensei_phase2.launch.py",
                "launch/sensei_phase3.launch.py",
                "launch/sensei_phase4.launch.py",
                "launch/sensei_phase5.launch.py",
                "launch/sensei_control_min.launch.py",
                "launch/sensei_rviz.launch.py",
            ],
        ),
        ("share/" + package_name + "/config", ["config/ros2_controllers.yaml"]),
        (
            "share/" + package_name + "/urdf",
            [
                "urdf/sensei_arm.urdf.xacro",
                "urdf/sensei_base.urdf.xacro",
                "urdf/sensei_arm_left.urdf.xacro",
                "urdf/sensei_arm_right.urdf.xacro",
                "urdf/sensei_leg_left.urdf.xacro",
                "urdf/sensei_leg_right.urdf.xacro",
                "urdf/sensei_head.urdf.xacro",
                "urdf/sensei_full.urdf.xacro",
            ],
        ),
        ("share/" + package_name + "/rviz", ["rviz/sensei.rviz"]),
        ("share/" + package_name + "/worlds", ["worlds/empty.sdf"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="karate_robot",
    maintainer_email="dev@karate-robot.local",
    description="Simulation scaffolding for the Karate Robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "test_left_arm_publisher=karate_robot_sim.test_left_arm_publisher:main",
        ],
    },
)
