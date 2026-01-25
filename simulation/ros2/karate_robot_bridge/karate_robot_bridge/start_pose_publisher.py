#!/usr/bin/env python3
"""
Publish a single desired_pose to seed a starting state.
Bushido note: the first stance sets the spirit of the kata.
"""

from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class StartPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("start_pose_publisher")
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
        self.declare_parameter("pose_mode", "kamae")
        self.declare_parameter("publish_delay_sec", 0.2)

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.pose_mode = str(self.get_parameter("pose_mode").value)
        self.publish_delay_sec = float(self.get_parameter("publish_delay_sec").value)

        self.publisher = self.create_publisher(Float64MultiArray, "desired_pose", 10)
        self.timer = self.create_timer(self.publish_delay_sec, self.publish_once)
        self.published = False

    def build_pose(self) -> List[float]:
        if self.pose_mode != "kamae":
            return [0.0] * len(self.joint_names)
        pose = [0.0] * len(self.joint_names)
        overrides = {
            "l_shoulder_pan": 0.3,
            "l_shoulder_lift": 0.4,
            "l_elbow_flex": -0.2,
            "r_shoulder_pan": -0.2,
            "r_shoulder_lift": 0.1,
            "r_elbow_flex": -1.0,
            "l_hip_pitch": 0.5,
            "l_knee_pitch": -0.9,
            "l_ankle_pitch": 0.4,
            "r_hip_pitch": 0.5,
            "r_knee_pitch": -0.9,
            "r_ankle_pitch": 0.4,
            "neck_pan": 0.4,
            "neck_tilt": -0.1,
        }
        for idx, name in enumerate(self.joint_names):
            if name in overrides:
                pose[idx] = overrides[name]
        return pose

    def publish_once(self) -> None:
        if self.published:
            return
        msg = Float64MultiArray()
        msg.data = self.build_pose()
        self.publisher.publish(msg)
        self.published = True
        self.get_logger().info("Published start pose (%s).", self.pose_mode)
        self.timer.cancel()
        self.destroy_node()
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = StartPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
