#!/usr/bin/env python3
"""
Strategist node that closes the loop with simulation.
Bushido note: form is practiced until intent and safety align.
"""

import math
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String, UInt8
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def load_state_machine():
    """Load strategist state machine, falling back to a local definition."""
    try:
        from strategist.state_machine import State, StateMachine  # type: ignore

        return State, StateMachine
    except Exception:
        repo_root = Path(__file__).resolve().parents[5]
        strategist_path = repo_root / "strategist" / "state_machine.py"
        if strategist_path.exists():
            import importlib.util

            spec = importlib.util.spec_from_file_location("state_machine", strategist_path)
            module = importlib.util.module_from_spec(spec)  # type: ignore
            if spec and spec.loader:
                spec.loader.exec_module(module)  # type: ignore
                return module.State, module.StateMachine  # type: ignore

    from enum import Enum

    class State(str, Enum):
        MOKUSO = "MOKUSO"
        REI = "REI"
        KAMAE = "KAMAE"
        MIMIC = "MIMIC"
        SAFETY_HALT = "SAFETY_HALT"

    class StateMachine:
        def __init__(self, identity_gate=None):
            self.state = State.MOKUSO
            self.identity_gate = identity_gate or (lambda: False)

        def can_enter_mimic(self) -> bool:
            return bool(self.identity_gate())

        def transition(self, next_state: State) -> State:
            if self.state == State.SAFETY_HALT:
                return self.state
            if next_state == State.MIMIC and not self.can_enter_mimic():
                return self.state
            self.state = next_state
            return self.state

        def safety_halt(self) -> State:
            self.state = State.SAFETY_HALT
            return self.state

    return State, StateMachine


State, StateMachine = load_state_machine()


class StrategistNode(Node):
    def __init__(self) -> None:
        super().__init__("strategist_node")

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
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("identity_override", True)
        self.declare_parameter("mokuso_sec", 2.0)
        self.declare_parameter("rei_sec", 2.0)
        self.declare_parameter("kamae_sec", 2.0)
        self.declare_parameter("mimic_cycle_sec", 6.0)

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.identity_override = bool(self.get_parameter("identity_override").value)
        self.mokuso_sec = float(self.get_parameter("mokuso_sec").value)
        self.rei_sec = float(self.get_parameter("rei_sec").value)
        self.kamae_sec = float(self.get_parameter("kamae_sec").value)
        self.mimic_cycle_sec = float(self.get_parameter("mimic_cycle_sec").value)

        self.identity_gate_value = False
        self.safety_halt_latch = 0
        self.state_started = time.monotonic()
        self.last_joint_positions: Dict[str, float] = {name: 0.0 for name in self.joint_names}

        self.state_machine = StateMachine(identity_gate=self.identity_gate_ok)

        self.pose_mokuso = {name: 0.0 for name in self.joint_names}
        self.pose_rei = {
            "neck_tilt": -0.25,
            "l_hip_pitch": 0.2,
            "r_hip_pitch": 0.2,
        }
        self.pose_kamae = {
            "l_shoulder_lift": 0.4,
            "r_shoulder_lift": 0.4,
            "l_elbow_flex": -0.7,
            "r_elbow_flex": -0.7,
            "l_knee_pitch": 0.3,
            "r_knee_pitch": 0.3,
            "neck_tilt": -0.05,
        }

        self.desired_pose_pub = self.create_publisher(Float64MultiArray, "desired_pose", 10)
        self.state_pub = self.create_publisher(String, "strategist/state", 10)
        self.left_arm_pub = self.create_publisher(
            JointTrajectory, "left_arm_controller/joint_trajectory", 10
        )

        self.create_subscription(JointState, "joint_states", self.on_joint_state, 10)
        self.create_subscription(UInt8, "register_map/safety_halt_latch", self.on_latch, 10)
        self.create_subscription(Bool, "identity_gate", self.on_identity_gate, 10)

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.tick)
        self.get_logger().info("Strategist node online.")

    def identity_gate_ok(self) -> bool:
        return bool(self.identity_override or self.identity_gate_value)

    def on_identity_gate(self, msg: Bool) -> None:
        self.identity_gate_value = bool(msg.data)

    def on_latch(self, msg: UInt8) -> None:
        self.safety_halt_latch = int(msg.data)

    def on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in self.last_joint_positions:
                self.last_joint_positions[name] = float(position)

    def tick(self) -> None:
        if self.safety_halt_latch:
            if self.state_machine.state != State.SAFETY_HALT:
                self.state_machine.safety_halt()
                self.state_started = time.monotonic()
            self.publish_state(self.hold_pose())
            return

        if self.state_machine.state == State.SAFETY_HALT:
            self.state_machine.transition(State.MOKUSO)
            self.state_started = time.monotonic()

        self.advance_state()

        if self.state_machine.state == State.MIMIC:
            pose = self.mimic_pose()
        elif self.state_machine.state == State.REI:
            pose = self.apply_pose(self.pose_rei)
        elif self.state_machine.state == State.KAMAE:
            pose = self.apply_pose(self.pose_kamae)
        else:
            pose = self.apply_pose(self.pose_mokuso)

        self.publish_state(pose)

    def advance_state(self) -> None:
        now = time.monotonic()
        elapsed = now - self.state_started
        state = self.state_machine.state

        if state == State.MOKUSO and elapsed >= self.mokuso_sec:
            self.state_machine.transition(State.REI)
            self.state_started = now
        elif state == State.REI and elapsed >= self.rei_sec:
            self.state_machine.transition(State.KAMAE)
            self.state_started = now
        elif state == State.KAMAE and elapsed >= self.kamae_sec:
            before = self.state_machine.state
            self.state_machine.transition(State.MIMIC)
            if self.state_machine.state != before:
                self.state_started = now
            else:
                self.state_started = now
        elif state == State.MIMIC:
            if not self.identity_gate_ok():
                self.state_machine.transition(State.KAMAE)
                self.state_started = now
            elif self.mimic_cycle_sec > 0 and elapsed >= self.mimic_cycle_sec:
                self.state_machine.transition(State.KAMAE)
                self.state_started = now

    def hold_pose(self) -> List[float]:
        return [self.last_joint_positions[name] for name in self.joint_names]

    def apply_pose(self, overrides: Dict[str, float]) -> List[float]:
        pose = [0.0] * len(self.joint_names)
        for idx, name in enumerate(self.joint_names):
            pose[idx] = overrides.get(name, 0.0)
        return pose

    def mimic_pose(self) -> List[float]:
        base = self.apply_pose(self.pose_kamae)
        now = time.monotonic()
        phase = (now - self.state_started) * (2 * math.pi / max(self.mimic_cycle_sec, 0.1))
        for name, amplitude, offset in (
            ("l_shoulder_pan", 0.4, 0.0),
            ("l_shoulder_lift", 0.3, 0.8),
            ("l_elbow_flex", 0.4, 1.2),
        ):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                base[idx] = base[idx] + amplitude * math.sin(phase + offset)
        return base

    def publish_state(self, pose: List[float]) -> None:
        msg = Float64MultiArray()
        msg.data = list(pose)
        self.desired_pose_pub.publish(msg)

        state_msg = String()
        state_msg.data = str(self.state_machine.state)
        self.state_pub.publish(state_msg)

        self.publish_left_arm(pose)

    def publish_left_arm(self, pose: List[float]) -> None:
        joint_subset = ["l_shoulder_pan", "l_shoulder_lift", "l_elbow_flex"]
        positions: List[float] = []
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
        self.left_arm_pub.publish(traj)


def main() -> None:
    rclpy.init()
    node = StrategistNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
