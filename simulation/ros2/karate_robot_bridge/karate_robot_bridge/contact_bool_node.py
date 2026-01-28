#!/usr/bin/env python3
"""
Convert Gazebo contact arrays into simple Bool flags.
"""

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool


class ContactBoolNode(Node):
    def __init__(self) -> None:
        super().__init__("contact_bool")

        self.declare_parameter("contact_topic", "contact/left_foot_raw")
        self.declare_parameter("bool_topic", "contact/left_foot")
        self.declare_parameter("publish_rate_hz", 10.0)

        contact_topic = str(self.get_parameter("contact_topic").value)
        bool_topic = str(self.get_parameter("bool_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.publisher = self.create_publisher(Bool, bool_topic, 10)
        self.subscription = self.create_subscription(
            Contacts, contact_topic, self.on_contacts, 10
        )
        self.last_state = False
        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 0.1), self.publish_state
        )

        self.get_logger().info(
            f"Contact bool bridge listening on {contact_topic} -> {bool_topic}"
        )

    def on_contacts(self, msg: Contacts) -> None:
        contacts = getattr(msg, "contacts", None)
        if contacts is None:
            contacts = getattr(msg, "contact", None)
        self.last_state = bool(contacts) and len(contacts) > 0

    def publish_state(self) -> None:
        self.publisher.publish(Bool(data=self.last_state))


def main() -> None:
    rclpy.init()
    node = ContactBoolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
