#!/usr/bin/env python3
"""
Simple desired_pose publisher for simulation testing.
Bushido note: training forms are slow and deliberate before real technique.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class TestPosePublisher(Node):
    def __init__(self):
        super().__init__("test_pose_publisher")
        self.declare_parameter("amplitude", 0.4)
        self.declare_parameter("rate_hz", 10.0)
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
        self.amplitude = float(self.get_parameter("amplitude").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.publisher = self.create_publisher(Float64MultiArray, "desired_pose", 10)
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_pose)

    def publish_pose(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        base = self.amplitude * math.sin(elapsed)
        msg = Float64MultiArray()
        msg.data = [base] * len(self.joint_names)
        self.publisher.publish(msg)

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(self.joint_names)
        joint_state.position = list(msg.data)
        self.joint_state_pub.publish(joint_state)


def main() -> None:
    rclpy.init()
    node = TestPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
