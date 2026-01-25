#!/usr/bin/env python3
"""
Training policy runner for Phase 6.
Bushido note: a calm mind repeats with purpose.
"""

import csv
import json
import math
import random
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64MultiArray, Int32, String


class TrainingAgentNode(Node):
    def __init__(self) -> None:
        super().__init__("training_agent")

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
        self.declare_parameter("policy_mode", "sine")
        self.declare_parameter("base_pose_mode", "kamae")
        self.declare_parameter("action_scale", 0.4)
        self.declare_parameter("action_clip", 1.2)
        self.declare_parameter("sine_frequency_hz", 0.25)
        self.declare_parameter(
            "active_joints",
            [
                "r_hip_pitch",
                "r_knee_pitch",
                "r_ankle_pitch",
                "l_shoulder_pan",
                "l_shoulder_lift",
                "l_elbow_flex",
                "r_shoulder_pan",
                "r_shoulder_lift",
                "r_elbow_flex",
            ],
        )
        self.declare_parameter("reset_on_done", True)
        self.declare_parameter("log_path", "")
        self.declare_parameter("log_every_sec", 2.0)

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.joint_index: Dict[str, int] = {
            name: idx for idx, name in enumerate(self.joint_names)
        }
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.policy_mode = str(self.get_parameter("policy_mode").value)
        self.base_pose_mode = str(self.get_parameter("base_pose_mode").value)
        self.action_scale = float(self.get_parameter("action_scale").value)
        self.action_clip = float(self.get_parameter("action_clip").value)
        self.sine_frequency_hz = float(self.get_parameter("sine_frequency_hz").value)
        self.active_joints = list(self.get_parameter("active_joints").value)
        self.reset_on_done = bool(self.get_parameter("reset_on_done").value)
        self.log_path = str(self.get_parameter("log_path").value)
        self.log_every_sec = float(self.get_parameter("log_every_sec").value)

        self.latest_reward: Optional[float] = None
        self.latest_done = False
        self.latest_episode = 0
        self.last_reset_sent = False
        self.last_log_time = time.monotonic()
        self.episode_reward = 0.0
        self.episode_steps = 0
        self.schema_payload: Optional[Dict[str, object]] = None

        self.base_pose = (
            self.build_kamae_pose()
            if self.base_pose_mode == "kamae"
            else [0.0] * len(self.joint_names)
        )
        self.current_action = list(self.base_pose)
        self.start_time = time.monotonic()

        self.action_pub = self.create_publisher(
            Float64MultiArray, "training/action", 10
        )
        self.reset_pub = self.create_publisher(Bool, "training/reset", 10)

        self.create_subscription(
            Float64MultiArray, "training/observation", self.on_observation, 10
        )
        self.create_subscription(Float32, "training/reward", self.on_reward, 10)
        self.create_subscription(Bool, "training/done", self.on_done, 10)
        self.create_subscription(Int32, "training/episode", self.on_episode, 10)
        self.create_subscription(String, "training/schema", self.on_schema, 10)

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.tick)
        self.csv_file: Optional[Path] = None
        self.csv_writer: Optional[csv.writer] = None
        self.open_log()
        self.get_logger().info("Training agent online.")

    def build_kamae_pose(self) -> List[float]:
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
        return kamae

    def on_observation(self, _: Float64MultiArray) -> None:
        pass

    def on_reward(self, msg: Float32) -> None:
        self.latest_reward = float(msg.data)
        self.episode_reward += self.latest_reward

    def on_done(self, msg: Bool) -> None:
        self.latest_done = bool(msg.data)

    def on_episode(self, msg: Int32) -> None:
        self.latest_episode = int(msg.data)

    def on_schema(self, msg: String) -> None:
        if self.schema_payload is not None:
            return
        try:
            self.schema_payload = json.loads(msg.data)
            self.get_logger().info("Training schema received.")
        except json.JSONDecodeError:
            self.get_logger().warn("Training schema is not valid JSON.")

    def open_log(self) -> None:
        if not self.log_path:
            return
        log_path = Path(self.log_path).expanduser()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = log_path
        file_handle = open(log_path, "w", newline="")
        self.csv_writer = csv.writer(file_handle)
        self.csv_writer.writerow(
            [
                "timestamp",
                "episode",
                "step",
                "reward",
                "episode_reward",
                "done",
                "action_mean_abs",
            ]
        )
        self.get_logger().info("Logging to %s", str(log_path))

    def tick(self) -> None:
        if self.latest_done and self.reset_on_done and not self.last_reset_sent:
            self.reset_pub.publish(Bool(data=True))
            self.last_reset_sent = True
            self.current_action = list(self.base_pose)
            self.episode_reward = 0.0
            self.episode_steps = 0
            return

        if not self.latest_done:
            self.last_reset_sent = False

        now = time.monotonic() - self.start_time
        if self.policy_mode == "hold":
            self.current_action = list(self.base_pose)
        elif self.policy_mode == "random_walk":
            self.current_action = self.apply_random_walk(self.current_action)
        else:
            self.current_action = self.apply_sine(now)

        self.action_pub.publish(Float64MultiArray(data=self.current_action))
        self.episode_steps += 1
        self.maybe_log()

    def maybe_log(self) -> None:
        if not self.csv_writer:
            return
        now = time.monotonic()
        if (now - self.last_log_time) < self.log_every_sec:
            return
        self.last_log_time = now
        mean_abs = sum(abs(v) for v in self.current_action) / max(len(self.current_action), 1)
        self.csv_writer.writerow(
            [
                time.time(),
                self.latest_episode,
                self.episode_steps,
                self.latest_reward if self.latest_reward is not None else 0.0,
                self.episode_reward,
                int(self.latest_done),
                mean_abs,
            ]
        )

    def apply_sine(self, t_sec: float) -> List[float]:
        action = list(self.base_pose)
        omega = 2.0 * math.pi * self.sine_frequency_hz
        for idx, name in enumerate(self.active_joints):
            joint_index = self.joint_index.get(name)
            if joint_index is None:
                continue
            phase = idx * 0.3
            delta = self.action_scale * math.sin(omega * t_sec + phase)
            action[joint_index] = self.clamp(action[joint_index] + delta)
        return action

    def apply_random_walk(self, current: List[float]) -> List[float]:
        action = list(current)
        for name in self.active_joints:
            joint_index = self.joint_index.get(name)
            if joint_index is None:
                continue
            step = random.uniform(-0.05, 0.05) * self.action_scale
            action[joint_index] = self.clamp(action[joint_index] + step)
        return action

    def clamp(self, value: float) -> float:
        return max(-self.action_clip, min(self.action_clip, value))


def main() -> None:
    rclpy.init()
    node = TrainingAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
