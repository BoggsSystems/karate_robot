#!/usr/bin/env python3
"""
Reflex guard for Phase 2 safety validation.
Bushido note: when force exceeds justice, motion yields to safety.
"""

import time
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int16MultiArray, UInt8
from visualization_msgs.msg import Marker


class ReflexGuard(Node):
    def __init__(self) -> None:
        super().__init__("reflex_guard")

        self.declare_parameter("kime_intensity", 2500)
        self.declare_parameter("current_spike_trip", 6500)
        self.declare_parameter("spike_after_sec", 12.0)
        self.declare_parameter("joint_count", 14)
        self.declare_parameter("marker_frame", "base_link")

        self.kime_intensity = int(self.get_parameter("kime_intensity").value)
        self.current_spike_trip = int(self.get_parameter("current_spike_trip").value)
        self.spike_after_sec = float(self.get_parameter("spike_after_sec").value)
        self.joint_count = int(self.get_parameter("joint_count").value)
        self.marker_frame = str(self.get_parameter("marker_frame").value)

        self.target_angles: List[int] = [0] * self.joint_count
        self.actual_angles: List[int] = [0] * self.joint_count
        self.output_angles: List[int] = [0] * self.joint_count

        self.safety_halt_latch = 0
        self._last_latch = 0
        self.start_time = time.monotonic()

        self.create_subscription(
            Int16MultiArray, "register_map/target_angles", self.on_target, 10
        )
        self.create_subscription(
            Int16MultiArray, "register_map/actual_angles", self.on_actual, 10
        )
        self.create_subscription(
            Bool, "register_map/reset_safety_halt", self.on_reset, 10
        )

        self.output_pub = self.create_publisher(
            Int16MultiArray, "register_map/output_angles", 10
        )
        self.latch_pub = self.create_publisher(UInt8, "register_map/safety_halt_latch", 10)
        self.marker_pub = self.create_publisher(Marker, "reflex_guard/status_marker", 10)

        self.timer = self.create_timer(0.05, self.evaluate_reflex)
        self.get_logger().info("Reflex guard online.")

    def on_target(self, msg: Int16MultiArray) -> None:
        if len(msg.data) != self.joint_count:
            return
        self.target_angles = list(msg.data)

    def on_actual(self, msg: Int16MultiArray) -> None:
        if len(msg.data) != self.joint_count:
            return
        self.actual_angles = list(msg.data)

    def evaluate_reflex(self) -> None:
        if self.safety_halt_latch != 0:
            self.output_angles = list(self.actual_angles)
            self.publish_state()
            return

        now = time.monotonic()
        simulated_spike = (
            self.spike_after_sec > 0.0 and (now - self.start_time) >= self.spike_after_sec
        )

        for idx in range(self.joint_count):
            target = self.target_angles[idx]
            actual = self.actual_angles[idx]
            simulated_torque = abs(target - actual)

            if simulated_spike or simulated_torque > self.current_spike_trip:
                self.safety_halt_latch = 1
                self.output_angles = list(self.actual_angles)
                if self._last_latch == 0:
                    self.get_logger().warn("Safety halt latched.")
                self.publish_state()
                return

            if simulated_torque > self.kime_intensity:
                self.output_angles[idx] = actual
            else:
                self.output_angles[idx] = target

        self.publish_state()

    def publish_state(self) -> None:
        self.output_pub.publish(Int16MultiArray(data=self.output_angles))
        self.latch_pub.publish(UInt8(data=self.safety_halt_latch))
        self.publish_marker()
        self._last_latch = self.safety_halt_latch

    def on_reset(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.safety_halt_latch = 0
        self._last_latch = 0
        self.start_time = time.monotonic()
        self.get_logger().info("Safety halt reset.")

    def publish_marker(self) -> None:
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "reflex_guard"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.25
        marker.scale.z = 0.08
        if self.safety_halt_latch:
            marker.text = "SAFETY HALT"
            marker.color.r = 0.9
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 1.0
        else:
            marker.text = "Safety OK"
            marker.color.r = 0.5
            marker.color.g = 0.9
            marker.color.b = 0.5
            marker.color.a = 0.9
        self.marker_pub.publish(marker)


def main() -> None:
    rclpy.init()
    node = ReflexGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
