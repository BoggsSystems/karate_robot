#!/usr/bin/env python3
"""
Training environment adapter for Phase 4.
Bushido note: repetition without reflection is empty; logging keeps the mirror.
"""

import csv
import json
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
        self.declare_parameter("publish_schema", True)
        self.declare_parameter("schema_version", 1)
        self.declare_parameter("goal_hip_pitch", 0.7)
        self.declare_parameter("goal_knee_pitch", -0.2)
        self.declare_parameter("goal_ankle_pitch", 0.1)
        self.declare_parameter("reward_hip_weight", 0.6)
        self.declare_parameter("reward_knee_weight", 0.9)
        self.declare_parameter("reward_ankle_weight", 0.2)
        self.declare_parameter("reward_safety_penalty", -1.0)
        self.declare_parameter("reward_velocity_penalty", 0.0)
        self.declare_parameter("success_reward", 1.0)
        self.declare_parameter("success_knee_threshold", -0.25)
        self.declare_parameter("success_hip_delta", 0.15)

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.episode_timeout_sec = float(self.get_parameter("episode_timeout_sec").value)
        self.log_path = str(self.get_parameter("log_path").value)
        self.publish_schema = bool(self.get_parameter("publish_schema").value)
        self.schema_version = int(self.get_parameter("schema_version").value)
        self.goal_hip_pitch = float(self.get_parameter("goal_hip_pitch").value)
        self.goal_knee_pitch = float(self.get_parameter("goal_knee_pitch").value)
        self.goal_ankle_pitch = float(self.get_parameter("goal_ankle_pitch").value)
        self.reward_hip_weight = float(self.get_parameter("reward_hip_weight").value)
        self.reward_knee_weight = float(self.get_parameter("reward_knee_weight").value)
        self.reward_ankle_weight = float(self.get_parameter("reward_ankle_weight").value)
        self.reward_safety_penalty = float(
            self.get_parameter("reward_safety_penalty").value
        )
        self.reward_velocity_penalty = float(
            self.get_parameter("reward_velocity_penalty").value
        )
        self.success_reward = float(self.get_parameter("success_reward").value)
        self.success_knee_threshold = float(
            self.get_parameter("success_knee_threshold").value
        )
        self.success_hip_delta = float(self.get_parameter("success_hip_delta").value)

        self.joint_positions: Dict[str, float] = {name: 0.0 for name in self.joint_names}
        self.joint_velocities: Dict[str, float] = {name: 0.0 for name in self.joint_names}
        self.episode_start_positions: Dict[str, float] = dict(self.joint_positions)
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
        self.schema_pub = self.create_publisher(String, "training/schema", 1)

        self.create_subscription(JointState, "joint_states", self.on_joint_state, 10)
        self.create_subscription(UInt8, "register_map/safety_halt_latch", self.on_latch, 10)
        self.create_subscription(String, "strategist/state", self.on_state, 10)
        self.create_subscription(Float64MultiArray, "training/action", self.on_action, 10)
        self.create_subscription(Bool, "training/reset", self.on_reset, 10)

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.tick)

        self.csv_file: Optional[Path] = None
        self.csv_writer: Optional[csv.writer] = None
        self.open_log()
        self.publish_training_schema()

        self.get_logger().info("Training environment online.")

    def build_training_schema(self) -> Dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "observations": {
                "layout": ["joint_positions", "safety_latch", "strategist_state_id"],
                "joint_units": "radians",
                "safety_latch": {
                    "type": "uint8",
                    "meaning": "0 = ok, 1 = safety halt latched",
                },
                "strategist_state_id": {
                    "order": ["MOKUSO", "REI", "KAMAE", "MIMIC", "SAFETY_HALT"],
                },
            },
            "actions": {
                "type": "float64_array",
                "layout": "joint_targets",
                "joint_units": "radians",
                "description": "absolute joint targets matching joint_names order",
            },
            "reward": {
                "hip_target": self.goal_hip_pitch,
                "knee_target": self.goal_knee_pitch,
                "ankle_target": self.goal_ankle_pitch,
                "weights": {
                    "hip": self.reward_hip_weight,
                    "knee": self.reward_knee_weight,
                    "ankle": self.reward_ankle_weight,
                },
                "safety_latch_penalty": self.reward_safety_penalty,
                "velocity_penalty": self.reward_velocity_penalty,
                "success_reward": self.success_reward,
            },
            "done": {
                "conditions": [
                    "safety_latch == 1",
                    "kick_goal reached",
                    "episode_timeout_sec elapsed (if > 0)",
                ],
            },
            "joint_names": list(self.joint_names),
        }

    def publish_training_schema(self) -> None:
        if not self.publish_schema:
            return
        schema = self.build_training_schema()
        self.schema_pub.publish(String(data=json.dumps(schema)))
        self.get_logger().info("Published training schema v%d.", self.schema_version)

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
        if msg.velocity:
            for name, velocity in zip(msg.name, msg.velocity):
                if name in self.joint_velocities:
                    self.joint_velocities[name] = float(velocity)

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
        self.episode_start_positions = dict(self.joint_positions)
        self.reset_pub.publish(Bool(data=True))
        self.get_logger().info("Episode reset.")

    def tick(self) -> None:
        observation = [self.joint_positions[name] for name in self.joint_names]
        state_id = float(self.state_to_id(self.state_name))
        observation.extend([float(self.safety_latch), state_id])
        self.observation_pub.publish(Float64MultiArray(data=observation))

        reward = Float32()
        reward.data = self.compute_reward()
        self.reward_pub.publish(reward)

        done = Bool()
        elapsed = time.monotonic() - self.episode_start
        done.data = bool(
            self.safety_latch
            or self.reached_kick_goal()
            or (self.episode_timeout_sec > 0 and elapsed >= self.episode_timeout_sec)
        )
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

    def get_joint(self, name: str) -> float:
        return float(self.joint_positions.get(name, 0.0))

    def get_joint_start(self, name: str) -> float:
        return float(self.episode_start_positions.get(name, 0.0))

    def reached_kick_goal(self) -> bool:
        hip = self.get_joint("r_hip_pitch")
        knee = self.get_joint("r_knee_pitch")
        hip_delta = hip - self.get_joint_start("r_hip_pitch")
        return hip_delta >= self.success_hip_delta and knee >= self.success_knee_threshold

    def compute_reward(self) -> float:
        if self.safety_latch:
            return self.reward_safety_penalty

        hip = self.get_joint("r_hip_pitch")
        knee = self.get_joint("r_knee_pitch")
        ankle = self.get_joint("r_ankle_pitch")

        hip_term = -abs(self.goal_hip_pitch - hip) * self.reward_hip_weight
        knee_term = -abs(self.goal_knee_pitch - knee) * self.reward_knee_weight
        ankle_term = -abs(self.goal_ankle_pitch - ankle) * self.reward_ankle_weight

        reward = hip_term + knee_term + ankle_term
        if self.reached_kick_goal():
            reward += self.success_reward

        if self.reward_velocity_penalty > 0.0:
            vel_mag = sum(abs(v) for v in self.joint_velocities.values())
            reward -= self.reward_velocity_penalty * vel_mag

        return float(reward)


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
