#!/usr/bin/env python3
"""ROS 2 one-shot publisher: ``std_msgs/String`` JSON payload for V3.0.1 blocked_passage."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .blocked_passage_json_v301 import TRANSPORT_TOPIC_DEFAULT


class BlockedPassageTransportSenderNode(Node):
    """Publishes ``payload_json`` once after ``publish_once_after_sec`` (default 0.5s)."""

    def __init__(self) -> None:
        super().__init__("blocked_passage_transport_sender")
        self.declare_parameter("topic", TRANSPORT_TOPIC_DEFAULT)
        self.declare_parameter("payload_json", "")
        self.declare_parameter("publish_once_after_sec", 0.5)

        topic = str(self.get_parameter("topic").value or TRANSPORT_TOPIC_DEFAULT)
        self._payload = str(self.get_parameter("payload_json").value or "")
        delay = float(self.get_parameter("publish_once_after_sec").value)

        self._pub = self.create_publisher(String, topic, 10)
        self._published = False
        self._timer = self.create_timer(max(delay, 0.01), self._try_publish)
        self.get_logger().info("will publish one String to %s after %.3fs" % (topic, delay))

    def _try_publish(self) -> None:
        if self._published:
            self._timer.cancel()
            return
        self._published = True
        self._timer.cancel()
        if not self._payload.strip():
            self.get_logger().error("payload_json is empty; not publishing")
            return
        msg = String()
        msg.data = self._payload
        self._pub.publish(msg)
        self.get_logger().info("published blocked_passage transport payload (%d bytes)" % len(msg.data))


def main() -> None:
    rclpy.init()
    try:
        node = BlockedPassageTransportSenderNode()
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
