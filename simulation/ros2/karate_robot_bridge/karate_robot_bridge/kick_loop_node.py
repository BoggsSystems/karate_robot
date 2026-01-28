#!/usr/bin/env python3
"""
Periodic front-kick loop (right leg).
Bushido note: timing turns effort into technique.
"""

from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class KickLoopNode(Node):
    def __init__(self) -> None:
        super().__init__("kick_loop")

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
        self.declare_parameter("update_rate_hz", 30.0)
        self.declare_parameter("kick_period_sec", 3.0)
        self.declare_parameter("chamber_sec", 0.5)
        self.declare_parameter("extend_sec", 0.3)
        self.declare_parameter("retract_sec", 0.2)
        self.declare_parameter("kick_side", "right")

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.kick_period_sec = float(self.get_parameter("kick_period_sec").value)
        self.chamber_sec = float(self.get_parameter("chamber_sec").value)
        self.extend_sec = float(self.get_parameter("extend_sec").value)
        self.retract_sec = float(self.get_parameter("retract_sec").value)
        self.kick_side = str(self.get_parameter("kick_side").value).lower()
        if self.kick_side not in ("left", "right"):
            self.get_logger().warn(
                "Invalid kick_side '%s'; defaulting to 'right'.", self.kick_side
            )
            self.kick_side = "right"

        self.desired_pose_pub = self.create_publisher(
            Float64MultiArray, "desired_pose", 10
        )

        self.poses = self.build_poses()
        self.phase_names = ["hold", "chamber", "extend", "retract"]
        self.phase_index = 0
        self.phase_start = self.get_clock().now().nanoseconds * 1e-9

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.tick)
        self.get_logger().info(
            f"Kick loop online (period {self.kick_period_sec:.2f}s, side {self.kick_side})."
        )
        self.get_logger().info("Kick phases: hold -> chamber -> extend -> retract")

    def build_poses(self) -> Dict[str, List[float]]:
        kamae = [0.0] * len(self.joint_names)
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
                kamae[idx] = overrides[name]

        hip, knee, ankle = self.kick_joint_names()
        chamber = list(kamae)
        self.set_joint(chamber, hip, 0.8)
        self.set_joint(chamber, knee, -0.6)
        self.set_joint(chamber, ankle, 0.2)

        extend = list(chamber)
        self.set_joint(extend, knee, 0.1)
        self.set_joint(extend, ankle, 0.25)

        retract = list(chamber)

        return {
            "hold": kamae,
            "chamber": chamber,
            "extend": extend,
            "retract": retract,
        }

    def set_joint(self, pose: List[float], joint_name: str, value: float) -> None:
        if joint_name in self.joint_names:
            pose[self.joint_names.index(joint_name)] = value

    def kick_joint_names(self) -> tuple[str, str, str]:
        prefix = "l" if self.kick_side == "left" else "r"
        return f"{prefix}_hip_pitch", f"{prefix}_knee_pitch", f"{prefix}_ankle_pitch"

    def phase_durations(self) -> List[float]:
        active = self.chamber_sec + self.extend_sec + self.retract_sec
        hold = max(self.kick_period_sec - active, 0.0)
        return [hold, self.chamber_sec, self.extend_sec, self.retract_sec]

    def tick(self) -> None:
        durations = self.phase_durations()
        current_phase = self.phase_names[self.phase_index]
        pose = self.poses[current_phase]
        self.publish_pose(pose)

        if durations[self.phase_index] <= 0:
            self.advance_phase()
            return

        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.phase_start
        if elapsed >= durations[self.phase_index]:
            self.advance_phase()

    def advance_phase(self) -> None:
        self.phase_index = (self.phase_index + 1) % len(self.phase_names)
        self.phase_start = self.get_clock().now().nanoseconds * 1e-9
        current_phase = self.phase_names[self.phase_index]
        if current_phase == "extend":
            self.get_logger().info("Kick initiated: extend phase.")
        elif current_phase == "chamber":
            self.get_logger().debug("Kick phase: chamber.")
        elif current_phase == "retract":
            self.get_logger().debug("Kick phase: retract.")

    def publish_pose(self, pose: List[float]) -> None:
        msg = Float64MultiArray()
        msg.data = list(pose)
        self.desired_pose_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = KickLoopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
