#!/usr/bin/env python3
"""
Publish JointState from desired_pose as a visualization fallback.
Bushido note: when the forge is quiet, the form still flows.
"""

from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class DesiredPoseJointState(Node):
    def __init__(self) -> None:
        super().__init__("desired_pose_joint_state")

        self.declare_parameter(
            "joint_names",
            [
                "l_shoulder_pan",
                "l_shoulder_lift",
                "l_elbow_flex",
                "r_shoulder_pan",
                "r_shoulder_lift",
                "r_elbow_flex",
                "l_hip_pitch",
                "l_knee_pitch",
                "l_ankle_pitch",
                "r_hip_pitch",
                "r_knee_pitch",
                "r_ankle_pitch",
                "neck_pan",
                "neck_tilt",
            ],
        )
        self.declare_parameter("publish_rate_hz", 10.0)
        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.last_positions = [0.0] * len(self.joint_names)

        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(
            Float64MultiArray, "desired_pose", self.on_desired_pose, 10
        )

        self.publish_joint_state(self.last_positions)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)
        self.get_logger().info("Desired pose joint state fallback online.")

    def on_desired_pose(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warn(
                "desired_pose length mismatch; expected %d got %d",
                len(self.joint_names),
                len(msg.data),
            )
            return

        self.last_positions = list(msg.data)
        self.publish_joint_state(self.last_positions)

    def publish_joint_state(self, positions: List[float]) -> None:
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(self.joint_names)
        joint_state.position = list(positions)
        self.publisher.publish(joint_state)

    def on_timer(self) -> None:
        self.publish_joint_state(self.last_positions)


def main() -> None:
    rclpy.init()
    node = DesiredPoseJointState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
