#!/usr/bin/env python3
"""
Policy runner for evaluating trained walking policies.
Bushido note: practice reveals the path.
"""

import csv
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64MultiArray, Int32


class PolicyRunnerNode(Node):
    def __init__(self) -> None:
        super().__init__("policy_runner")

        self.declare_parameter("model_path", "/tmp/karate_robot_walk_policy")
        self.declare_parameter("algo", "PPO")
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("action_clip", 1.2)
        self.declare_parameter("deterministic", True)
        self.declare_parameter("reset_on_done", True)
        self.declare_parameter("log_path", "")
        self.declare_parameter("log_every_sec", 2.0)
        self.declare_parameter("odom_topic", "odom")

        self.model_path = str(self.get_parameter("model_path").value)
        self.algo = str(self.get_parameter("algo").value).upper()
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.action_clip = float(self.get_parameter("action_clip").value)
        self.deterministic = bool(self.get_parameter("deterministic").value)
        self.reset_on_done = bool(self.get_parameter("reset_on_done").value)
        self.log_path = str(self.get_parameter("log_path").value)
        self.log_every_sec = float(self.get_parameter("log_every_sec").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)

        self.model = self.load_model(self.algo, self.model_path)
        self.latest_observation: Optional[np.ndarray] = None
        self.latest_reward = 0.0
        self.latest_done = False
        self.latest_episode = -1
        self.last_reset_sent = False

        self.episode_reward = 0.0
        self.episode_steps = 0
        self.episode_start_time = time.monotonic()
        self.episode_start_position: Optional[Tuple[float, float]] = None
        self.current_position: Optional[Tuple[float, float]] = None
        self.last_log_time = time.monotonic()

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
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.tick)

        self.csv_writer: Optional[csv.writer] = None
        self.open_log()
        self.get_logger().info("Policy runner online.")

    def load_model(self, algo: str, model_path: str):
        try:
            from stable_baselines3 import PPO, SAC
        except ImportError as exc:
            raise RuntimeError(
                "stable-baselines3 is required. Install with: pip install stable-baselines3 gymnasium"
            ) from exc

        if algo == "SAC":
            return SAC.load(model_path)
        if algo == "PPO":
            return PPO.load(model_path)
        raise RuntimeError(f"Unsupported algo '{algo}'. Use PPO or SAC.")

    def open_log(self) -> None:
        if not self.log_path:
            return
        log_path = Path(self.log_path).expanduser()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        file_handle = open(log_path, "w", newline="")
        self.csv_writer = csv.writer(file_handle)
        self.csv_writer.writerow(
            [
                "timestamp",
                "episode",
                "steps",
                "reward_total",
                "duration_sec",
                "distance_m",
                "reward_avg",
            ]
        )
        self.get_logger().info(f"Logging to {log_path}")

    def on_observation(self, msg: Float64MultiArray) -> None:
        self.latest_observation = np.asarray(msg.data, dtype=np.float32)

    def on_reward(self, msg: Float32) -> None:
        self.latest_reward = float(msg.data)
        self.episode_reward += self.latest_reward

    def on_done(self, msg: Bool) -> None:
        self.latest_done = bool(msg.data)

    def on_episode(self, msg: Int32) -> None:
        episode = int(msg.data)
        if episode != self.latest_episode:
            self.start_episode(episode)
        self.latest_episode = episode

    def on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self.current_position = (float(position.x), float(position.y))
        if self.episode_start_position is None:
            self.episode_start_position = self.current_position

    def start_episode(self, episode: int) -> None:
        self.latest_episode = episode
        self.episode_reward = 0.0
        self.episode_steps = 0
        self.episode_start_time = time.monotonic()
        self.episode_start_position = self.current_position
        self.last_reset_sent = False

    def finalize_episode(self) -> None:
        duration = time.monotonic() - self.episode_start_time
        distance = 0.0
        if self.current_position and self.episode_start_position:
            dx = self.current_position[0] - self.episode_start_position[0]
            dy = self.current_position[1] - self.episode_start_position[1]
            distance = float((dx * dx + dy * dy) ** 0.5)
        avg_reward = self.episode_reward / max(self.episode_steps, 1)
        if self.csv_writer:
            self.csv_writer.writerow(
                [
                    time.time(),
                    self.latest_episode,
                    self.episode_steps,
                    self.episode_reward,
                    duration,
                    distance,
                    avg_reward,
                ]
            )
        self.get_logger().info(
            f"Episode {self.latest_episode} done: steps={self.episode_steps} reward={self.episode_reward:.3f} distance={distance:.3f} avg={avg_reward:.3f}"
        )

    def tick(self) -> None:
        if self.latest_observation is None:
            return

        if self.latest_done and self.reset_on_done and not self.last_reset_sent:
            self.finalize_episode()
            self.reset_pub.publish(Bool(data=True))
            self.last_reset_sent = True
            return

        if not self.latest_done:
            self.last_reset_sent = False

        action, _ = self.model.predict(
            self.latest_observation, deterministic=self.deterministic
        )
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if self.action_clip > 0:
            action = np.clip(action, -self.action_clip, self.action_clip)
        self.action_pub.publish(Float64MultiArray(data=action.tolist()))
        self.episode_steps += 1
        self.maybe_log_progress()

    def maybe_log_progress(self) -> None:
        if not self.csv_writer:
            return
        now = time.monotonic()
        if (now - self.last_log_time) < self.log_every_sec:
            return
        self.last_log_time = now
        self.csv_writer.writerow(
            [
                time.time(),
                self.latest_episode,
                self.episode_steps,
                self.episode_reward,
                time.monotonic() - self.episode_start_time,
                "",
                "",
            ]
        )


def main() -> None:
    rclpy.init()
    node = PolicyRunnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
