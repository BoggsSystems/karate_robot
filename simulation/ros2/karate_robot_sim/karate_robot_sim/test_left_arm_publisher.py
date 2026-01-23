import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class LeftArmTestPublisher(Node):
    def __init__(self) -> None:
        super().__init__("left_arm_test_publisher")
        self._publisher = self.create_publisher(
            JointTrajectory, "left_arm_controller/joint_trajectory", 10
        )
        self._safety_latch = 0
        self._last_positions = [0.0, 0.0, 0.0]
        self._joint_index = {
            "l_shoulder_pan": 0,
            "l_shoulder_lift": 1,
            "l_elbow_flex": 2,
        }

        self.create_subscription(UInt8, "register_map/safety_halt_latch", self._on_latch, 10)
        self.create_subscription(JointState, "joint_states", self._on_joint_state, 10)
        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(0.5, self._publish_command)

    def _publish_command(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self._start_time).nanoseconds * 1e-9
        phase = elapsed * 0.7

        if self._safety_latch != 0:
            positions = list(self._last_positions)
        else:
            shoulder_pan = 0.4 * math.sin(phase)
            shoulder_lift = 0.5 * math.sin(phase + 0.8)
            elbow_flex = -0.6 + 0.4 * math.sin(phase + 1.6)
            positions = [shoulder_pan, shoulder_lift, elbow_flex]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1

        msg = JointTrajectory()
        msg.joint_names = ["l_shoulder_pan", "l_shoulder_lift", "l_elbow_flex"]
        msg.points = [point]

        self._publisher.publish(msg)

    def _on_latch(self, msg: UInt8) -> None:
        self._safety_latch = int(msg.data)

    def _on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            idx = self._joint_index.get(name)
            if idx is None:
                continue
            if len(self._last_positions) <= idx:
                continue
            self._last_positions[idx] = float(position)


def main() -> None:
    rclpy.init()
    node = LeftArmTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
