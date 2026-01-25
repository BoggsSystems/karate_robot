#!/usr/bin/env python3
"""
ROS 2 RL trainer using Stable-Baselines3 (PPO/SAC).
Bushido note: the way is walked, one step at a time.
"""

import json
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64MultiArray, Int32, String


@dataclass
class TrainingSample:
    observation: np.ndarray
    reward: float
    done: bool


class TrainingBridge(Node):
    def __init__(self) -> None:
        super().__init__("rl_trainer_bridge")

        self.declare_parameter("action_clip", 1.2)
        self.declare_parameter("observation_timeout_sec", 2.0)
        self.declare_parameter("schema_timeout_sec", 2.0)

        self.action_clip = float(self.get_parameter("action_clip").value)
        self.observation_timeout_sec = float(
            self.get_parameter("observation_timeout_sec").value
        )
        self.schema_timeout_sec = float(
            self.get_parameter("schema_timeout_sec").value
        )

        self.latest_observation: Optional[np.ndarray] = None
        self.latest_reward = 0.0
        self.latest_done = False
        self.latest_episode = 0
        self.schema_payload: Optional[dict] = None

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

        self.get_logger().info("RL trainer bridge online.")

    def on_observation(self, msg: Float64MultiArray) -> None:
        self.latest_observation = np.asarray(msg.data, dtype=np.float32)

    def on_reward(self, msg: Float32) -> None:
        self.latest_reward = float(msg.data)

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

    def wait_for_schema(self) -> Optional[dict]:
        if self.schema_payload is not None:
            return self.schema_payload
        deadline = time.monotonic() + self.schema_timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.schema_payload is not None:
                return self.schema_payload
        return None

    def wait_for_observation(self) -> Optional[np.ndarray]:
        deadline = time.monotonic() + self.observation_timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_observation is not None:
                return self.latest_observation
        return None

    def sample(self, action: np.ndarray) -> Optional[TrainingSample]:
        clipped = np.clip(action, -self.action_clip, self.action_clip)
        self.action_pub.publish(Float64MultiArray(data=clipped.tolist()))
        obs = self.wait_for_observation()
        if obs is None:
            return None
        return TrainingSample(obs, self.latest_reward, self.latest_done)

    def reset(self) -> Optional[np.ndarray]:
        self.reset_pub.publish(Bool(data=True))
        self.latest_done = False
        return self.wait_for_observation()


def build_env(bridge: TrainingBridge):
    try:
        import gymnasium as gym
        from gymnasium import spaces
    except ImportError as exc:
        raise RuntimeError(
            "gymnasium is required. Install with: pip install gymnasium stable-baselines3"
        ) from exc

    schema = bridge.wait_for_schema()
    if schema:
        joint_names = schema.get("joint_names", [])
        obs_size = len(joint_names) + 2
        action_size = len(joint_names)
    else:
        obs_size = 16
        action_size = 14

    class RosTrainingEnv(gym.Env):
        metadata = {"render_modes": []}

        def __init__(self) -> None:
            super().__init__()
            self.observation_space = spaces.Box(
                low=-np.inf, high=np.inf, shape=(obs_size,), dtype=np.float32
            )
            self.action_space = spaces.Box(
                low=-bridge.action_clip,
                high=bridge.action_clip,
                shape=(action_size,),
                dtype=np.float32,
            )

        def reset(self, *, seed: Optional[int] = None, options=None):
            super().reset(seed=seed)
            obs = bridge.reset()
            if obs is None:
                obs = np.zeros((obs_size,), dtype=np.float32)
            return obs, {}

        def step(self, action):
            sample = bridge.sample(action)
            if sample is None:
                obs = np.zeros((obs_size,), dtype=np.float32)
                return obs, 0.0, False, False, {}
            terminated = bool(sample.done)
            truncated = False
            return sample.observation, sample.reward, terminated, truncated, {}

    return RosTrainingEnv()


def main() -> None:
    rclpy.init()
    bridge = TrainingBridge()

    bridge.declare_parameter("algo", "PPO")
    bridge.declare_parameter("total_timesteps", 20000)
    bridge.declare_parameter("model_path", "/tmp/karate_robot_policy")
    bridge.declare_parameter("log_dir", "/tmp/karate_robot_rl")

    algo = str(bridge.get_parameter("algo").value).upper()
    total_timesteps = int(bridge.get_parameter("total_timesteps").value)
    model_path = str(bridge.get_parameter("model_path").value)
    log_dir = str(bridge.get_parameter("log_dir").value)

    env = build_env(bridge)

    try:
        from stable_baselines3 import PPO, SAC
    except ImportError as exc:
        raise RuntimeError(
            "stable-baselines3 is required. Install with: pip install stable-baselines3 gymnasium"
        ) from exc

    if algo == "SAC":
        model = SAC("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)
    else:
        model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)

    model.learn(total_timesteps=total_timesteps)
    model.save(model_path)
    bridge.get_logger().info("Saved policy to %s", model_path)

    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
