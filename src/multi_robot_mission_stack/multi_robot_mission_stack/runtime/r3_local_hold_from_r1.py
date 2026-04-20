#!/usr/bin/env python3
"""
R3 bounded local robot-behavior consequence from accepted R1 degraded advisories.

Trigger source:
- /semantic/degraded_passage_p1_1 (std_msgs/String JSON)

Local reversible effect:
- for a short bounded window, publish zero Twist on one robot cmd_vel topic
  (default: /robot1_ns/cmd_vel)

This node adds no planner/coordinator logic, no replanning, and no multi-source fusion.
"""

from __future__ import annotations

import json
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


def is_valid_r1_degraded_payload(payload: str) -> bool:
    if not payload:
        return False
    try:
        obj: Any = json.loads(payload)
    except json.JSONDecodeError:
        return False
    if not isinstance(obj, dict):
        return False
    return str(obj.get("fact_type", "")).strip() == "degraded_passage" and bool(
        str(obj.get("belief_id", "")).strip()
    )


class R3LocalHoldFromR1(Node):
    def __init__(self) -> None:
        super().__init__("r3_local_hold_from_r1")
        self.declare_parameter("source_topic", "/semantic/degraded_passage_p1_1")
        self.declare_parameter("target_cmd_vel_topic", "/robot1_ns/cmd_vel")
        self.declare_parameter("hold_duration_sec", 2.0)
        self.declare_parameter("hold_publish_hz", 10.0)
        self.declare_parameter("min_retrigger_gap_sec", 2.0)

        self._source_topic = str(self.get_parameter("source_topic").value)
        self._target_cmd_vel_topic = str(self.get_parameter("target_cmd_vel_topic").value)
        self._hold_duration_sec = max(0.2, float(self.get_parameter("hold_duration_sec").value))
        self._hold_publish_hz = max(1.0, float(self.get_parameter("hold_publish_hz").value))
        self._min_retrigger_gap_sec = max(0.0, float(self.get_parameter("min_retrigger_gap_sec").value))

        self._pub = self.create_publisher(Twist, self._target_cmd_vel_topic, 10)
        self.create_subscription(String, self._source_topic, self._on_r1_degraded, 10)
        self.create_timer(1.0 / self._hold_publish_hz, self._tick)

        self._hold_until_mono_sec = 0.0
        self._last_hold_end_mono_sec = -1e9
        self._was_holding = False

        self.get_logger().info(
            "R3 local hold active: source=%s target=%s hold=%.2fs"
            % (self._source_topic, self._target_cmd_vel_topic, self._hold_duration_sec)
        )

    def _on_r1_degraded(self, msg: String) -> None:
        if not is_valid_r1_degraded_payload(msg.data):
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_hold_end_mono_sec < self._min_retrigger_gap_sec:
            return
        new_hold_until = now + self._hold_duration_sec
        if new_hold_until > self._hold_until_mono_sec:
            self._hold_until_mono_sec = new_hold_until
            self.get_logger().info(
                "R3 hold armed from R1 degraded advisory (target=%s until=%.3f)"
                % (self._target_cmd_vel_topic, self._hold_until_mono_sec)
            )

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        holding = now <= self._hold_until_mono_sec
        if holding:
            zero = Twist()
            self._pub.publish(zero)
            self._was_holding = True
            return
        if self._was_holding:
            self._last_hold_end_mono_sec = now
            self._was_holding = False
            self.get_logger().info("R3 hold released (target=%s)" % self._target_cmd_vel_topic)


def main() -> None:
    rclpy.init()
    try:
        node = R3LocalHoldFromR1()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
