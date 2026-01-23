#!/usr/bin/env python3
"""
Training environment adapter for Phase 4.
Bushido note: repetition without reflection is empty; logging keeps the mirror.
"""

import csv
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray, Int32, String, UInt8


class TrainingEnvNode(Node):
    def __init__(self) -> None:
        super().__init__("training_env")

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
        self.declare_parameter("log_path", "")
        self.declare_parameter("episode_timeout_sec", 20.0)

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.episode_timeout_sec = float(self.get_parameter("episode_timeout_sec").value)
        self.log_path = str(self.get_parameter("log_path").value)

        self.joint_positions: Dict[str, float] = {name: 0.0 for name in self.joint_names}
        self.safety_latch = 0
        self.state_name = "MOKUSO"
        self.episode_id = 0
        self.episode_start = time.monotonic()

        self.desired_pose_pub = self.create_publisher(
            Float64MultiArray, "desired_pose", 10
        )
        self.reset_pub = self.create_publisher(
            Bool, "register_map/reset_safety_halt", 10
        )
        self.observation_pub = self.create_publisher(
            Float64MultiArray, "training/observation", 10
        )
        self.reward_pub = self.create_publisher(Float32, "training/reward", 10)
        self.done_pub = self.create_publisher(Bool, "training/done", 10)
        self.episode_pub = self.create_publisher(Int32, "training/episode", 10)

        self.create_subscription(JointState, "joint_states", self.on_joint_state, 10)
        self.create_subscription(UInt8, "register_map/safety_halt_latch", self.on_latch, 10)
        self.create_subscription(String, "strategist/state", self.on_state, 10)
        self.create_subscription(Float64MultiArray, "training/action", self.on_action, 10)
        self.create_subscription(Bool, "training/reset", self.on_reset, 10)

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.tick)

        self.csv_file: Optional[Path] = None
        self.csv_writer: Optional[csv.writer] = None
        self.open_log()

        self.get_logger().info("Training environment online.")

    def open_log(self) -> None:
        if not self.log_path:
            return
        log_path = Path(self.log_path).expanduser()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = log_path
        file_handle = open(log_path, "w", newline="")
        self.csv_writer = csv.writer(file_handle)
        header = ["timestamp", "episode", "state", "safety_latch"] + [
            f"joint_{name}" for name in self.joint_names
        ]
        self.csv_writer.writerow(header)
        self.get_logger().info("Logging to %s", str(log_path))

    def on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_positions:
                self.joint_positions[name] = float(position)

    def on_latch(self, msg: UInt8) -> None:
        self.safety_latch = int(msg.data)

    def on_state(self, msg: String) -> None:
        self.state_name = msg.data

    def on_action(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warn(
                "Action length mismatch; expected %d got %d",
                len(self.joint_names),
                len(msg.data),
            )
            return
        self.desired_pose_pub.publish(msg)

    def on_reset(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.episode_id += 1
        self.episode_start = time.monotonic()
        self.reset_pub.publish(Bool(data=True))
        self.get_logger().info("Episode reset.")

    def tick(self) -> None:
        observation = [self.joint_positions[name] for name in self.joint_names]
        state_id = float(self.state_to_id(self.state_name))
        observation.extend([float(self.safety_latch), state_id])
        self.observation_pub.publish(Float64MultiArray(data=observation))

        reward = Float32()
        reward.data = -1.0 if self.safety_latch else 0.0
        self.reward_pub.publish(reward)

        done = Bool()
        elapsed = time.monotonic() - self.episode_start
        done.data = bool(self.safety_latch or (self.episode_timeout_sec > 0 and elapsed >= self.episode_timeout_sec))
        self.done_pub.publish(done)

        self.episode_pub.publish(Int32(data=self.episode_id))

        if self.csv_writer:
            row = [
                time.time(),
                self.episode_id,
                self.state_name,
                self.safety_latch,
                *[self.joint_positions[name] for name in self.joint_names],
            ]
            self.csv_writer.writerow(row)

    @staticmethod
    def state_to_id(state_name: str) -> int:
        order = ["MOKUSO", "REI", "KAMAE", "MIMIC", "SAFETY_HALT"]
        if state_name in order:
            return order.index(state_name)
        return -1


def main() -> None:
    rclpy.init()
    node = TrainingEnvNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
