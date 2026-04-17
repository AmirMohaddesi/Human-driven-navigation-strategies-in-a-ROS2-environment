#!/usr/bin/env python3
"""Publish one ``std_msgs/String`` message (payload = stdin) to a topic, then exit."""

from __future__ import annotations

import sys
import time

import rclpy
from std_msgs.msg import String


def main() -> None:
    if len(sys.argv) != 2:
        print("usage: p1_1_publish_std_string_once.py /topic/name", file=sys.stderr)
        sys.exit(2)
    topic = sys.argv[1].strip()
    raw = sys.stdin.read()
    payload = raw.strip("\n\r")

    rclpy.init()
    node = rclpy.create_node("p1_1_publish_std_string_once")
    pub = node.create_publisher(String, topic, 10)
    deadline = time.time() + 2.0
    while pub.get_subscription_count() < 1 and time.time() < deadline:
        time.sleep(0.05)
    msg = String()
    msg.data = payload
    pub.publish(msg)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
