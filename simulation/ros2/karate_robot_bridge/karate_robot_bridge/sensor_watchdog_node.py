#!/usr/bin/env python3
"""
Watchdog that verifies odom and foot contact topics are publishing.
"""

import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


class SensorWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("sensor_watchdog")

        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("left_contact_topic", "contact/left_foot")
        self.declare_parameter("right_contact_topic", "contact/right_foot")
        self.declare_parameter("timeout_sec", 1.0)
        self.declare_parameter("report_period_sec", 2.0)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.left_contact_topic = str(self.get_parameter("left_contact_topic").value)
        self.right_contact_topic = str(self.get_parameter("right_contact_topic").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.report_period_sec = float(self.get_parameter("report_period_sec").value)

        self.last_odom_time: float | None = None
        self.last_left_time: float | None = None
        self.last_right_time: float | None = None
        self.last_left_value: bool | None = None
        self.last_right_value: bool | None = None
        self.last_report_time: float | None = None
        self.odom_count = 0
        self.left_count = 0
        self.right_count = 0

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(Bool, self.left_contact_topic, self.on_left_contact, 10)
        self.create_subscription(Bool, self.right_contact_topic, self.on_right_contact, 10)

        self.timer = self.create_timer(self.report_period_sec, self.report_status)
        self.get_logger().info(
            f"Watching topics: odom={self.odom_topic} "
            f"left={self.left_contact_topic} right={self.right_contact_topic}"
        )

    def on_odom(self, msg: Odometry) -> None:
        self.last_odom_time = time.monotonic()
        self.odom_count += 1

    def on_left_contact(self, msg: Bool) -> None:
        self.last_left_time = time.monotonic()
        self.last_left_value = bool(msg.data)
        self.left_count += 1

    def on_right_contact(self, msg: Bool) -> None:
        self.last_right_time = time.monotonic()
        self.last_right_value = bool(msg.data)
        self.right_count += 1

    def report_status(self) -> None:
        now = time.monotonic()
        elapsed = None if self.last_report_time is None else now - self.last_report_time
        odom_rate = self._rate(self.odom_count, elapsed)
        left_rate = self._rate(self.left_count, elapsed)
        right_rate = self._rate(self.right_count, elapsed)
        self._report_topic("odom", self.last_odom_time, now, rate_hz=odom_rate)
        self._report_topic(
            "left_contact",
            self.last_left_time,
            now,
            last_value=self.last_left_value,
            rate_hz=left_rate,
        )
        self._report_topic(
            "right_contact",
            self.last_right_time,
            now,
            last_value=self.last_right_value,
            rate_hz=right_rate,
        )
        self.odom_count = 0
        self.left_count = 0
        self.right_count = 0
        self.last_report_time = now

    def _report_topic(
        self,
        label: str,
        last_time: float | None,
        now: float,
        last_value: bool | None = None,
        rate_hz: float | None = None,
    ) -> None:
        if last_time is None:
            self.get_logger().warn(f"{label}: no messages received yet")
            return
        age = now - last_time
        rate_text = "" if rate_hz is None else f", {rate_hz:.1f} hz"
        if age > self.timeout_sec:
            self.get_logger().warn(f"{label}: stale ({age:.2f}s{rate_text})")
        else:
            if last_value is None:
                self.get_logger().info(f"{label}: ok ({age:.2f}s{rate_text})")
            else:
                self.get_logger().info(
                    f"{label}: ok ({age:.2f}s{rate_text}, {last_value})"
                )

    @staticmethod
    def _rate(count: int, elapsed: float | None) -> float | None:
        if elapsed is None or elapsed <= 0.0:
            return None
        return count / elapsed


def main() -> None:
    rclpy.init()
    node = SensorWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
