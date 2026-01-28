#!/usr/bin/env python3
"""
Training environment adapter for Phase 4.
Bushido note: repetition without reflection is empty; logging keeps the mirror.
"""

import csv
import json
import math
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray, Int32, String, UInt8

from karate_robot_bridge.domain_randomization import (
    DomainRandomizer,
    RandomizationConfig,
    create_conservative_randomizer,
    create_aggressive_randomizer,
)


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
        self.declare_parameter("schema_version", 2)
        self.declare_parameter("reward_mode", "walk")
        self.declare_parameter("reward_profile", "custom")
        self.declare_parameter("curriculum_enabled", False)
        self.declare_parameter("curriculum_stage_episodes", 50)
        self.declare_parameter(
            "curriculum_profiles", ["stable", "baseline", "forward"]
        )
        self.declare_parameter("action_smoothing_alpha", 0.5)
        self.declare_parameter("action_position_min", -1.2)
        self.declare_parameter("action_position_max", 1.2)
        self.declare_parameter("action_max_delta_per_sec", 2.0)
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("left_contact_topic", "contact/left_foot")
        self.declare_parameter("right_contact_topic", "contact/right_foot")
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
        self.declare_parameter("walk_frequency_hz", 1.2)
        self.declare_parameter("walk_hip_swing", 0.5)
        self.declare_parameter("walk_knee_lift", 0.6)
        self.declare_parameter("walk_ankle_lift", 0.25)
        self.declare_parameter("walk_hip_weight", 0.5)
        self.declare_parameter("walk_knee_weight", 0.4)
        self.declare_parameter("walk_ankle_weight", 0.2)
        self.declare_parameter("walk_symmetry_weight", 0.15)
        self.declare_parameter("walk_energy_penalty", 0.01)
        self.declare_parameter("walk_ramp_sec", 5.0)
        self.declare_parameter("walk_min_scale", 0.3)
        self.declare_parameter("walk_forward_velocity_target", 0.3)
        self.declare_parameter("walk_forward_velocity_weight", 0.6)
        self.declare_parameter("walk_upright_weight", 0.2)
        self.declare_parameter("walk_height_target", 0.9)
        self.declare_parameter("walk_height_weight", 0.0)
        self.declare_parameter("walk_fall_pitch_rad", 0.9)
        self.declare_parameter("walk_fall_roll_rad", 0.9)
        self.declare_parameter("walk_min_base_height", 0.0)
        self.declare_parameter("walk_contact_weight", 0.2)
        self.declare_parameter("walk_air_timeout_sec", 0.6)
        self.declare_parameter("walk_success_sec", 6.0)
        self.declare_parameter("walk_error_threshold", 0.35)
        
        # Domain randomization for sim-to-real transfer
        self.declare_parameter("domain_randomization", "conservative")  # off, conservative, aggressive

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.episode_timeout_sec = float(self.get_parameter("episode_timeout_sec").value)
        self.log_path = str(self.get_parameter("log_path").value)
        self.publish_schema = bool(self.get_parameter("publish_schema").value)
        self.schema_version = int(self.get_parameter("schema_version").value)
        self.reward_mode = str(self.get_parameter("reward_mode").value).lower()
        self.reward_profile = str(self.get_parameter("reward_profile").value).lower()
        self.curriculum_enabled = bool(
            self.get_parameter("curriculum_enabled").value
        )
        self.curriculum_stage_episodes = int(
            self.get_parameter("curriculum_stage_episodes").value
        )
        self.curriculum_profiles = [
            str(item).lower()
            for item in self.get_parameter("curriculum_profiles").value
        ]
        self.action_smoothing_alpha = float(
            self.get_parameter("action_smoothing_alpha").value
        )
        self.action_position_min = float(
            self.get_parameter("action_position_min").value
        )
        self.action_position_max = float(
            self.get_parameter("action_position_max").value
        )
        self.action_max_delta_per_sec = float(
            self.get_parameter("action_max_delta_per_sec").value
        )
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.left_contact_topic = str(self.get_parameter("left_contact_topic").value)
        self.right_contact_topic = str(self.get_parameter("right_contact_topic").value)
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
        self.walk_frequency_hz = float(self.get_parameter("walk_frequency_hz").value)
        self.walk_hip_swing = float(self.get_parameter("walk_hip_swing").value)
        self.walk_knee_lift = float(self.get_parameter("walk_knee_lift").value)
        self.walk_ankle_lift = float(self.get_parameter("walk_ankle_lift").value)
        self.walk_hip_weight = float(self.get_parameter("walk_hip_weight").value)
        self.walk_knee_weight = float(self.get_parameter("walk_knee_weight").value)
        self.walk_ankle_weight = float(self.get_parameter("walk_ankle_weight").value)
        self.walk_symmetry_weight = float(
            self.get_parameter("walk_symmetry_weight").value
        )
        self.walk_energy_penalty = float(
            self.get_parameter("walk_energy_penalty").value
        )
        self.walk_ramp_sec = float(self.get_parameter("walk_ramp_sec").value)
        self.walk_min_scale = float(self.get_parameter("walk_min_scale").value)
        self.walk_forward_velocity_target = float(
            self.get_parameter("walk_forward_velocity_target").value
        )
        self.walk_forward_velocity_weight = float(
            self.get_parameter("walk_forward_velocity_weight").value
        )
        self.walk_upright_weight = float(
            self.get_parameter("walk_upright_weight").value
        )
        self.walk_height_target = float(
            self.get_parameter("walk_height_target").value
        )
        self.walk_height_weight = float(
            self.get_parameter("walk_height_weight").value
        )
        self.walk_fall_pitch_rad = float(
            self.get_parameter("walk_fall_pitch_rad").value
        )
        self.walk_fall_roll_rad = float(
            self.get_parameter("walk_fall_roll_rad").value
        )
        self.walk_min_base_height = float(
            self.get_parameter("walk_min_base_height").value
        )
        self.walk_contact_weight = float(
            self.get_parameter("walk_contact_weight").value
        )
        self.walk_air_timeout_sec = float(
            self.get_parameter("walk_air_timeout_sec").value
        )
        self.walk_success_sec = float(self.get_parameter("walk_success_sec").value)
        self.walk_error_threshold = float(
            self.get_parameter("walk_error_threshold").value
        )
        
        # Initialize domain randomization for sim-to-real transfer
        domain_rand_mode = str(self.get_parameter("domain_randomization").value).lower()
        if domain_rand_mode == "aggressive":
            self.domain_randomizer = create_aggressive_randomizer()
            self.get_logger().info("Domain randomization: AGGRESSIVE mode enabled")
        elif domain_rand_mode == "conservative":
            self.domain_randomizer = create_conservative_randomizer()
            self.get_logger().info("Domain randomization: CONSERVATIVE mode enabled")
        else:
            self.domain_randomizer = DomainRandomizer(RandomizationConfig(enabled=False))
            self.get_logger().info("Domain randomization: DISABLED")

        self.joint_positions: Dict[str, float] = {name: 0.0 for name in self.joint_names}
        self.joint_velocities: Dict[str, float] = {name: 0.0 for name in self.joint_names}
        self.episode_start_positions: Dict[str, float] = dict(self.joint_positions)
        self.last_action: Optional[List[float]] = None
        self.base_height = 0.0
        self.base_roll = 0.0
        self.base_pitch = 0.0
        self.base_yaw = 0.0
        self.base_lin_x = 0.0
        self.base_lin_y = 0.0
        self.base_ang_z = 0.0
        self.odom_received = False
        self.left_contact = 0
        self.right_contact = 0
        self.contact_received = False
        self.last_ground_contact_time: Optional[float] = None
        self.safety_latch = 0
        self.state_name = "MOKUSO"
        self.episode_id = 0
        self.episode_start = time.monotonic()
        self.reward_profiles: Optional[dict] = None

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
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(Bool, self.left_contact_topic, self.on_left_contact, 10)
        self.create_subscription(Bool, self.right_contact_topic, self.on_right_contact, 10)
        self.create_subscription(UInt8, "register_map/safety_halt_latch", self.on_latch, 10)
        self.create_subscription(String, "strategist/state", self.on_state, 10)
        self.create_subscription(Float64MultiArray, "training/action", self.on_action, 10)
        self.create_subscription(Bool, "training/reset", self.on_reset, 10)

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.tick)

        self.apply_reward_profile(self.reward_profile)
        self.csv_file: Optional[Path] = None
        self.csv_writer: Optional[csv.writer] = None
        self.open_log()
        self.publish_training_schema()

        self.get_logger().info("Training environment online.")

    def load_reward_profiles(self) -> dict:
        if self.reward_profiles is not None:
            return self.reward_profiles
        try:
            package_share = Path(get_package_share_directory("karate_robot_bridge"))
            profile_path = package_share / "config" / "walk_reward_profiles.yaml"
            profiles = yaml.safe_load(profile_path.read_text()) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warn(f"Failed to load reward profiles: {exc}")
            profiles = {}
        self.reward_profiles = profiles
        return profiles

    def apply_reward_profile(self, profile_name: str) -> None:
        if profile_name in ("", "custom"):
            return
        profiles = self.load_reward_profiles()
        profile = profiles.get(profile_name)
        if not isinstance(profile, dict):
            self.get_logger().warn(
                "Reward profile '%s' not found in %s",
                profile_name,
                "walk_reward_profiles.yaml",
            )
            return

        self.get_logger().info(f"Applying reward profile '{profile_name}'.")
        mapping = {
            "frequency_hz": "walk_frequency_hz",
            "hip_swing": "walk_hip_swing",
            "knee_lift": "walk_knee_lift",
            "ankle_lift": "walk_ankle_lift",
            "energy_penalty": "walk_energy_penalty",
            "ramp_sec": "walk_ramp_sec",
            "min_scale": "walk_min_scale",
            "forward_velocity_target": "walk_forward_velocity_target",
            "forward_velocity_weight": "walk_forward_velocity_weight",
            "upright_weight": "walk_upright_weight",
            "height_target": "walk_height_target",
            "height_weight": "walk_height_weight",
            "fall_pitch_rad": "walk_fall_pitch_rad",
            "fall_roll_rad": "walk_fall_roll_rad",
            "min_base_height": "walk_min_base_height",
            "contact_weight": "walk_contact_weight",
            "air_timeout_sec": "walk_air_timeout_sec",
            "success_sec": "walk_success_sec",
            "error_threshold": "walk_error_threshold",
        }
        for key, attr in mapping.items():
            if key in profile:
                setattr(self, attr, float(profile[key]))

        weights = profile.get("weights", {})
        if isinstance(weights, dict):
            if "hip" in weights:
                self.walk_hip_weight = float(weights["hip"])
            if "knee" in weights:
                self.walk_knee_weight = float(weights["knee"])
            if "ankle" in weights:
                self.walk_ankle_weight = float(weights["ankle"])
            if "symmetry" in weights:
                self.walk_symmetry_weight = float(weights["symmetry"])

    def advance_curriculum(self) -> None:
        if not self.curriculum_profiles:
            return
        stage = 0
        if self.curriculum_stage_episodes > 0:
            stage = self.episode_id // self.curriculum_stage_episodes
        index = min(stage, len(self.curriculum_profiles) - 1)
        next_profile = self.curriculum_profiles[index]
        if next_profile != self.reward_profile:
            self.reward_profile = next_profile
            self.apply_reward_profile(self.reward_profile)
            self.publish_training_schema()

    def build_training_schema(self) -> Dict[str, object]:
        done_conditions = (
            [
                "safety_latch == 1",
                "fallen (roll/pitch/height)",
                "air_timeout_sec exceeded (if > 0)",
                "walk_goal reached",
                "episode_timeout_sec elapsed (if > 0)",
            ]
            if self.reward_mode == "walk"
            else [
                "safety_latch == 1",
                "kick_goal reached",
                "episode_timeout_sec elapsed (if > 0)",
            ]
        )
        return {
            "schema_version": self.schema_version,
            "observations": {
                "layout": [
                    "joint_positions",
                    "safety_latch",
                    "strategist_state_id",
                    "base_pose",
                    "base_twist",
                    "foot_contacts",
                ],
                "observation_fields": [
                    {"name": "joint_positions", "size": len(self.joint_names)},
                    {"name": "safety_latch", "size": 1},
                    {"name": "strategist_state_id", "size": 1},
                    {"name": "base_pose", "size": 4},
                    {"name": "base_twist", "size": 3},
                    {"name": "foot_contacts", "size": 2},
                ],
                "joint_units": "radians",
                "safety_latch": {
                    "type": "uint8",
                    "meaning": "0 = ok, 1 = safety halt latched",
                },
                "strategist_state_id": {
                    "order": ["MOKUSO", "REI", "KAMAE", "MIMIC", "SAFETY_HALT"],
                },
                "base_pose": {
                    "layout": ["height_z", "roll", "pitch", "yaw"],
                    "units": ["meters", "radians", "radians", "radians"],
                },
                "base_twist": {
                    "layout": ["linear_x", "linear_y", "angular_z"],
                    "units": ["m/s", "m/s", "rad/s"],
                },
                "foot_contacts": {
                    "layout": ["left", "right"],
                    "type": "bool",
                    "topics": {
                        "left": self.left_contact_topic,
                        "right": self.right_contact_topic,
                    },
                },
            },
            "actions": {
                "type": "float64_array",
                "layout": "joint_targets",
                "joint_units": "radians",
                "description": "absolute joint targets matching joint_names order",
                "smoothing_alpha": self.action_smoothing_alpha,
                "position_min": self.action_position_min,
                "position_max": self.action_position_max,
                "max_delta_per_sec": self.action_max_delta_per_sec,
            },
            "reward_mode": self.reward_mode,
            "reward_profile": self.reward_profile,
            "curriculum": {
                "enabled": self.curriculum_enabled,
                "stage_episodes": self.curriculum_stage_episodes,
                "profiles": list(self.curriculum_profiles),
                "active_profile": self.reward_profile,
            },
            "reward": {
                "kick": {
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
                "walk": {
                    "frequency_hz": self.walk_frequency_hz,
                    "hip_swing": self.walk_hip_swing,
                    "knee_lift": self.walk_knee_lift,
                    "ankle_lift": self.walk_ankle_lift,
                    "weights": {
                        "hip": self.walk_hip_weight,
                        "knee": self.walk_knee_weight,
                        "ankle": self.walk_ankle_weight,
                        "symmetry": self.walk_symmetry_weight,
                    },
                    "energy_penalty": self.walk_energy_penalty,
                    "ramp_sec": self.walk_ramp_sec,
                    "min_scale": self.walk_min_scale,
                    "forward_velocity_target": self.walk_forward_velocity_target,
                    "forward_velocity_weight": self.walk_forward_velocity_weight,
                    "upright_weight": self.walk_upright_weight,
                    "height_target": self.walk_height_target,
                    "height_weight": self.walk_height_weight,
                    "fall_pitch_rad": self.walk_fall_pitch_rad,
                    "fall_roll_rad": self.walk_fall_roll_rad,
                    "min_base_height": self.walk_min_base_height,
                    "contact_weight": self.walk_contact_weight,
                    "air_timeout_sec": self.walk_air_timeout_sec,
                    "success_sec": self.walk_success_sec,
                    "error_threshold": self.walk_error_threshold,
                },
            },
            "done": {
                "conditions": done_conditions,
            },
            "joint_names": list(self.joint_names),
        }

    def publish_training_schema(self) -> None:
        if not self.publish_schema:
            return
        schema = self.build_training_schema()
        self.schema_pub.publish(String(data=json.dumps(schema)))
        self.get_logger().info(f"Published training schema v{self.schema_version}.")

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
        self.get_logger().info(f"Logging to {log_path}")

    def on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_positions:
                self.joint_positions[name] = float(position)
        if msg.velocity:
            for name, velocity in zip(msg.name, msg.velocity):
                if name in self.joint_velocities:
                    self.joint_velocities[name] = float(velocity)

    def on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.base_height = float(position.z)
        self.base_roll, self.base_pitch, self.base_yaw = quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.base_lin_x = float(msg.twist.twist.linear.x)
        self.base_lin_y = float(msg.twist.twist.linear.y)
        self.base_ang_z = float(msg.twist.twist.angular.z)
        self.odom_received = True

    def on_left_contact(self, msg: Bool) -> None:
        self.left_contact = 1 if msg.data else 0
        self.contact_received = True
        if msg.data:
            self.last_ground_contact_time = time.monotonic()

    def on_right_contact(self, msg: Bool) -> None:
        self.right_contact = 1 if msg.data else 0
        self.contact_received = True
        if msg.data:
            self.last_ground_contact_time = time.monotonic()

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
        alpha = max(0.0, min(1.0, self.action_smoothing_alpha))
        action = list(msg.data)
        if self.last_action is not None and alpha < 1.0:
            action = [
                alpha * current + (1.0 - alpha) * previous
                for current, previous in zip(action, self.last_action)
            ]
        action = self.apply_action_limits(action)
        
        # Apply domain randomization to actions (sim-to-real transfer)
        action = self.domain_randomizer.randomize_action(action)
        
        self.last_action = action
        self.desired_pose_pub.publish(Float64MultiArray(data=action))

    def on_reset(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.episode_id += 1
        if self.curriculum_enabled:
            self.advance_curriculum()
        self.episode_start = time.monotonic()
        self.episode_start_positions = dict(self.joint_positions)
        
        # Reset domain randomization for new episode
        episode_params = self.domain_randomizer.reset_episode()
        if episode_params:
            self.get_logger().debug(f"Episode randomization: {episode_params}")
        
        self.reset_pub.publish(Bool(data=True))
        self.get_logger().info("Episode reset.")

    def tick(self) -> None:
        observation = [self.joint_positions[name] for name in self.joint_names]
        state_id = float(self.state_to_id(self.state_name))
        observation.extend(
            [
                float(self.safety_latch),
                state_id,
                float(self.base_height),
                float(self.base_roll),
                float(self.base_pitch),
                float(self.base_yaw),
                float(self.base_lin_x),
                float(self.base_lin_y),
                float(self.base_ang_z),
                float(self.left_contact),
                float(self.right_contact),
            ]
        )
        
        # Apply domain randomization to observations (sim-to-real transfer)
        observation = self.domain_randomizer.randomize_observation(
            observation, joint_count=len(self.joint_names)
        )
        
        self.observation_pub.publish(Float64MultiArray(data=observation))

        reward = Float32()
        reward.data = self.compute_reward()
        self.reward_pub.publish(reward)

        done = Bool()
        elapsed = time.monotonic() - self.episode_start
        done.data = bool(
            self.safety_latch
            or self.is_fallen()
            or self.reached_goal()
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

    def reached_goal(self) -> bool:
        if self.reward_mode == "walk":
            return self.reached_walk_goal()
        return self.reached_kick_goal()

    def reached_kick_goal(self) -> bool:
        hip = self.get_joint("r_hip_pitch")
        knee = self.get_joint("r_knee_pitch")
        hip_delta = hip - self.get_joint_start("r_hip_pitch")
        return hip_delta >= self.success_hip_delta and knee >= self.success_knee_threshold

    def walk_phase(self) -> float:
        elapsed = time.monotonic() - self.episode_start
        return math.sin(2.0 * math.pi * self.walk_frequency_hz * elapsed)

    def apply_action_limits(self, action: List[float]) -> List[float]:
        clamped = [
            max(self.action_position_min, min(self.action_position_max, value))
            for value in action
        ]
        if self.action_max_delta_per_sec <= 0.0:
            return clamped
        max_delta = self.action_max_delta_per_sec / max(self.update_rate_hz, 1e-6)
        if self.last_action is None:
            last = [self.get_joint(name) for name in self.joint_names]
        else:
            last = self.last_action
        limited: List[float] = []
        for target, previous in zip(clamped, last):
            delta = max(-max_delta, min(max_delta, target - previous))
            limited.append(previous + delta)
        return limited

    def walk_scale(self) -> float:
        if self.walk_ramp_sec <= 0.0:
            return 1.0
        elapsed = time.monotonic() - self.episode_start
        progress = min(1.0, max(0.0, elapsed / self.walk_ramp_sec))
        return self.walk_min_scale + (1.0 - self.walk_min_scale) * progress

    def walk_targets(self) -> Dict[str, float]:
        phase = self.walk_phase()
        scale = self.walk_scale()
        left_phase = -phase

        def lift(p: float) -> float:
            return max(0.0, p)

        return {
            "l_hip_pitch": scale * self.walk_hip_swing * left_phase,
            "r_hip_pitch": scale * self.walk_hip_swing * phase,
            "l_knee_pitch": scale * self.walk_knee_lift * lift(left_phase),
            "r_knee_pitch": scale * self.walk_knee_lift * lift(phase),
            "l_ankle_pitch": scale * self.walk_ankle_lift * lift(left_phase),
            "r_ankle_pitch": scale * self.walk_ankle_lift * lift(phase),
        }

    def walk_error(self) -> float:
        targets = self.walk_targets()
        hip_error = abs(self.get_joint("l_hip_pitch") - targets["l_hip_pitch"]) + abs(
            self.get_joint("r_hip_pitch") - targets["r_hip_pitch"]
        )
        knee_error = abs(self.get_joint("l_knee_pitch") - targets["l_knee_pitch"]) + abs(
            self.get_joint("r_knee_pitch") - targets["r_knee_pitch"]
        )
        ankle_error = abs(
            self.get_joint("l_ankle_pitch") - targets["l_ankle_pitch"]
        ) + abs(self.get_joint("r_ankle_pitch") - targets["r_ankle_pitch"])
        symmetry = abs(self.get_joint("l_hip_pitch") + self.get_joint("r_hip_pitch"))

        return (
            self.walk_hip_weight * hip_error
            + self.walk_knee_weight * knee_error
            + self.walk_ankle_weight * ankle_error
            + self.walk_symmetry_weight * symmetry
        )

    def reached_walk_goal(self) -> bool:
        elapsed = time.monotonic() - self.episode_start
        return elapsed >= self.walk_success_sec and self.walk_error() <= self.walk_error_threshold

    def is_fallen(self) -> bool:
        if self.reward_mode != "walk" or not self.odom_received:
            return False
        orientation_fall = bool(
            abs(self.base_pitch) > self.walk_fall_pitch_rad
            or abs(self.base_roll) > self.walk_fall_roll_rad
            or self.base_height < self.walk_min_base_height
        )
        if orientation_fall:
            return True
        if not self.contact_received or self.walk_air_timeout_sec <= 0.0:
            return False
        if self.last_ground_contact_time is None:
            return False
        return (time.monotonic() - self.last_ground_contact_time) > self.walk_air_timeout_sec

    def gait_contact_score(self) -> float:
        if not self.contact_received:
            return 0.0
        expect_right = self.walk_phase() > 0.0
        if expect_right:
            stance = 1.0 if self.right_contact else -1.0
            swing = -0.5 if self.left_contact else 0.5
        else:
            stance = 1.0 if self.left_contact else -1.0
            swing = -0.5 if self.right_contact else 0.5
        return stance + swing

    def compute_reward(self) -> float:
        if self.safety_latch:
            return self.reward_safety_penalty

        if self.reward_mode == "walk":
            reward = -self.walk_error()
            if self.odom_received:
                vel_error = abs(
                    self.walk_forward_velocity_target - self.base_lin_x
                )
                reward -= self.walk_forward_velocity_weight * vel_error
                reward -= self.walk_upright_weight * (
                    abs(self.base_roll) + abs(self.base_pitch)
                )
                reward -= self.walk_height_weight * abs(
                    self.walk_height_target - self.base_height
                )
                if self.is_fallen():
                    return self.reward_safety_penalty
            if self.walk_contact_weight > 0.0:
                reward += self.walk_contact_weight * self.gait_contact_score()
            if self.reached_walk_goal():
                reward += self.success_reward
            if self.walk_energy_penalty > 0.0:
                vel_mag = sum(abs(v) for v in self.joint_velocities.values())
                reward -= self.walk_energy_penalty * vel_mag
            return float(reward)

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


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(_clamp(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


if __name__ == "__main__":
    main()
