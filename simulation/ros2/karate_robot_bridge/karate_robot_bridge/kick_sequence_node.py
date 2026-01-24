#!/usr/bin/env python3
"""
Phase 5: scripted right-leg front kick, then left straight punch and right elbow.
Bushido note: form precedes force; the kick is learned through disciplined arcs.
"""

import time
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class KickSequenceNode(Node):
    def __init__(self) -> None:
        super().__init__("kick_sequence")

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
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("hold_kamae_sec", 0.4)
        self.declare_parameter("chamber_sec", 0.3)
        self.declare_parameter("extend_sec", 0.2)
        self.declare_parameter("retract_sec", 0.2)
        self.declare_parameter("recover_sec", 0.3)
        self.declare_parameter("punch_extend_sec", 0.25)
        self.declare_parameter("punch_retract_sec", 0.25)
        self.declare_parameter("elbow_extend_sec", 0.25)
        self.declare_parameter("elbow_retract_sec", 0.25)

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.loop = bool(self.get_parameter("loop").value)

        self.durations = [
            float(self.get_parameter("hold_kamae_sec").value),
            float(self.get_parameter("chamber_sec").value),
            float(self.get_parameter("extend_sec").value),
            float(self.get_parameter("retract_sec").value),
            float(self.get_parameter("recover_sec").value),
            float(self.get_parameter("punch_extend_sec").value),
            float(self.get_parameter("punch_retract_sec").value),
            float(self.get_parameter("elbow_extend_sec").value),
            float(self.get_parameter("elbow_retract_sec").value),
        ]

        self.phase_names = [
            "kamae",
            "chamber",
            "extend",
            "retract",
            "recover",
            "left_punch_extend",
            "left_punch_retract",
            "right_elbow_extend",
            "right_elbow_retract",
        ]
        self.phase_index = 0
        self.phase_start = time.monotonic()

        self.desired_pose_pub = self.create_publisher(
            Float64MultiArray, "desired_pose", 10
        )
        self.right_leg_pub = self.create_publisher(
            JointTrajectory, "right_leg_controller/joint_trajectory", 10
        )

        self.poses = self.build_poses()
        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.tick)
        self.get_logger().info("Kick sequence online.")

    def build_poses(self) -> Dict[str, List[float]]:
        kamae = [0.0] * len(self.joint_names)
        overrides = {
            "l_shoulder_lift": 0.3,
            "l_elbow_flex": -0.6,
            "r_shoulder_lift": 0.3,
            "r_elbow_flex": -0.6,
            "l_hip_pitch": 0.2,
            "l_knee_pitch": 0.3,
            "l_ankle_pitch": 0.1,
            "r_hip_pitch": 0.2,
            "r_knee_pitch": 0.3,
            "r_ankle_pitch": 0.1,
            "neck_tilt": -0.05,
        }
        for idx, name in enumerate(self.joint_names):
            if name in overrides:
                kamae[idx] = overrides[name]

        chamber = list(kamae)
        self.set_joint(chamber, "r_hip_pitch", 0.7)
        self.set_joint(chamber, "r_knee_pitch", -0.6)
        self.set_joint(chamber, "r_ankle_pitch", 0.2)

        extend = list(chamber)
        self.set_joint(extend, "r_knee_pitch", 0.1)
        self.set_joint(extend, "r_ankle_pitch", 0.25)

        retract = list(chamber)
        recover = list(kamae)

        left_punch_extend = list(kamae)
        self.set_joint(left_punch_extend, "l_shoulder_pan", 0.2)
        self.set_joint(left_punch_extend, "l_shoulder_lift", 0.1)
        self.set_joint(left_punch_extend, "l_elbow_flex", -0.1)

        left_punch_retract = list(kamae)

        right_elbow_extend = list(kamae)
        self.set_joint(right_elbow_extend, "r_shoulder_pan", -0.25)
        self.set_joint(right_elbow_extend, "r_shoulder_lift", 0.2)
        self.set_joint(right_elbow_extend, "r_elbow_flex", -0.5)

        right_elbow_retract = list(kamae)

        return {
            "kamae": kamae,
            "chamber": chamber,
            "extend": extend,
            "retract": retract,
            "recover": recover,
            "left_punch_extend": left_punch_extend,
            "left_punch_retract": left_punch_retract,
            "right_elbow_extend": right_elbow_extend,
            "right_elbow_retract": right_elbow_retract,
        }

    def set_joint(self, pose: List[float], joint_name: str, value: float) -> None:
        if joint_name in self.joint_names:
            pose[self.joint_names.index(joint_name)] = value

    def tick(self) -> None:
        current_phase = self.phase_names[self.phase_index]
        pose = self.poses[current_phase]
        self.publish_pose(pose)
        self.publish_right_leg(pose)

        if self.durations[self.phase_index] <= 0:
            return

        if (time.monotonic() - self.phase_start) >= self.durations[self.phase_index]:
            if self.phase_index == len(self.phase_names) - 1:
                if self.loop:
                    self.phase_index = 0
                else:
                    return
            else:
                self.phase_index += 1
            self.phase_start = time.monotonic()

    def publish_pose(self, pose: List[float]) -> None:
        msg = Float64MultiArray()
        msg.data = list(pose)
        self.desired_pose_pub.publish(msg)

    def publish_right_leg(self, pose: List[float]) -> None:
        joint_subset = ["r_hip_pitch", "r_knee_pitch", "r_ankle_pitch"]
        positions = []
        for name in joint_subset:
            if name in self.joint_names:
                positions.append(pose[self.joint_names.index(name)])
            else:
                positions.append(0.0)

        traj = JointTrajectory()
        traj.joint_names = list(joint_subset)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1
        traj.points = [point]
        self.right_leg_pub.publish(traj)


def main() -> None:
    rclpy.init()
    node = KickSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
