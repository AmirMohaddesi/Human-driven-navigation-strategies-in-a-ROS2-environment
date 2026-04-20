#!/usr/bin/env python3
"""
R2 bounded visible consequence: render one RViz marker when R1 degraded advisories arrive.

Trigger source:
- /semantic/degraded_passage_p1_1 (std_msgs/String JSON from accepted R1 seam path)

Visible consequence:
- publish visualization_msgs/Marker to one topic for RViz display only

No control publishers, services, actions, or mission authority are added.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker


@dataclass(frozen=True)
class R2VisualEvent:
    belief_id: str
    fact_type: str
    location_ref: str
    ttl_sec: float


def parse_r1_degraded_event(payload: str, *, default_ttl_sec: float = 5.0) -> R2VisualEvent | None:
    """Parse one degraded_passage JSON payload for R2 visualization."""
    if not payload:
        return None
    try:
        raw: Any = json.loads(payload)
    except json.JSONDecodeError:
        return None
    if not isinstance(raw, dict):
        return None
    fact_type = str(raw.get("fact_type", "")).strip()
    if fact_type != "degraded_passage":
        return None
    belief_id = str(raw.get("belief_id", "")).strip()
    if not belief_id:
        return None
    location_ref = str(raw.get("location_ref", "")).strip()
    ttl_raw = raw.get("ttl_sec", default_ttl_sec)
    try:
        ttl_sec = float(ttl_raw)
    except (TypeError, ValueError):
        ttl_sec = float(default_ttl_sec)
    # Keep bounded and visually stable in RViz.
    if ttl_sec <= 0.0:
        ttl_sec = float(default_ttl_sec)
    ttl_sec = min(ttl_sec, 10.0)
    return R2VisualEvent(
        belief_id=belief_id,
        fact_type=fact_type,
        location_ref=location_ref,
        ttl_sec=ttl_sec,
    )


class R2DegradedVisualMarker(Node):
    def __init__(self) -> None:
        super().__init__("r2_degraded_visual_marker")
        self.declare_parameter("source_topic", "/semantic/degraded_passage_p1_1")
        self.declare_parameter("publish_topic", "/semantic/r2/degraded_marker")
        self.declare_parameter("default_ttl_sec", 5.0)
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("marker_x", 0.0)
        self.declare_parameter("marker_y", 0.0)
        self.declare_parameter("marker_z", 0.3)
        self.declare_parameter("marker_scale_m", 0.6)

        self._source_topic = str(self.get_parameter("source_topic").value)
        self._publish_topic = str(self.get_parameter("publish_topic").value)
        self._default_ttl_sec = float(self.get_parameter("default_ttl_sec").value)
        self._frame_id = str(self.get_parameter("marker_frame_id").value)
        self._marker_x = float(self.get_parameter("marker_x").value)
        self._marker_y = float(self.get_parameter("marker_y").value)
        self._marker_z = float(self.get_parameter("marker_z").value)
        self._marker_scale = float(self.get_parameter("marker_scale_m").value)

        self._pub = self.create_publisher(Marker, self._publish_topic, 10)
        self.create_subscription(String, self._source_topic, self._on_degraded, 10)
        self.get_logger().info(
            "R2 visual consequence active: %s -> %s (frame=%s)"
            % (self._source_topic, self._publish_topic, self._frame_id)
        )

    def _build_marker(self, event: R2VisualEvent) -> Marker:
        now_msg = self.get_clock().now().to_msg()
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = now_msg
        marker.ns = "r2_degraded_visible_consequence"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = self._marker_x
        marker.pose.position.y = self._marker_y
        marker.pose.position.z = self._marker_z
        marker.pose.orientation.w = 1.0
        marker.scale.z = max(self._marker_scale, 0.2)
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.95
        marker.lifetime.sec = int(event.ttl_sec)
        marker.lifetime.nanosec = int((event.ttl_sec - int(event.ttl_sec)) * 1e9)
        marker.text = "R2 degraded advisory active\nbelief_id=%s\n%s" % (
            event.belief_id,
            event.location_ref,
        )
        return marker

    def _on_degraded(self, msg: String) -> None:
        event = parse_r1_degraded_event(msg.data, default_ttl_sec=self._default_ttl_sec)
        if event is None:
            return
        marker = self._build_marker(event)
        self._pub.publish(marker)
        self.get_logger().info(
            "R2 visual marker emitted from R1 degraded advisory (belief_id=%s ttl=%.2fs)"
            % (event.belief_id, event.ttl_sec)
        )


def main() -> None:
    rclpy.init()
    try:
        node = R2DegradedVisualMarker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
