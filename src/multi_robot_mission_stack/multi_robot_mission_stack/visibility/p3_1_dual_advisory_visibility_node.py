"""
P3.1 — read-only stdout digest for dual advisory ``std_msgs/String`` transports (P1.1 topics).

Subscribe only. No service/action clients. No control publishers.
"""

from __future__ import annotations

import json
import sys
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Must match ``mission_bridge_headless_dual_advisory_p11.launch.py`` transport names.
TOPIC_BLOCKED = "/semantic/blocked_passage_p1_1"
TOPIC_DEGRADED = "/semantic/degraded_passage_p1_1"


def _utc_wall_stamp() -> str:
    dt = datetime.now(timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S.") + f"{dt.microsecond // 1000:03d}Z"


class P3DualAdvisoryVisibilityNode(Node):
    def __init__(self) -> None:
        super().__init__("p3_1_dual_advisory_visibility")
        self.create_subscription(String, TOPIC_BLOCKED, self._on_blocked, 10)
        self.create_subscription(String, TOPIC_DEGRADED, self._on_degraded, 10)
        self.get_logger().info(
            "P3.1 visibility-only: subscribed to %s and %s (no control path)." % (TOPIC_BLOCKED, TOPIC_DEGRADED)
        )

    def _print_digest(self, topic: str, data: str) -> None:
        recv = _utc_wall_stamp()
        if not data:
            print("[P3.1] recv=%s topic=%s EMPTY_PAYLOAD" % (recv, topic), flush=True)
            return
        try:
            obj = json.loads(data)
        except json.JSONDecodeError as e:
            head = data[:200].replace("\n", " ")
            print(
                "[P3.1] recv=%s topic=%s UNPARSEABLE_JSON err=%s raw_head=%r"
                % (recv, topic, e, head),
                flush=True,
            )
            return
        if not isinstance(obj, dict):
            print("[P3.1] recv=%s topic=%s JSON_NOT_OBJECT type=%s" % (recv, topic, type(obj).__name__), flush=True)
            return
        ft = obj.get("fact_type", "?")
        loc = obj.get("location_ref", "?")
        bid = obj.get("belief_id", "?")
        raw_head = data.replace("\n", " ")
        if len(raw_head) > 160:
            raw_head = raw_head[:160] + "…"
        print(
            "[P3.1] recv=%s topic=%s fact_type=%r location_ref=%r belief_id=%r raw_len=%d raw_head=%r"
            % (recv, topic, ft, loc, bid, len(data), raw_head),
            flush=True,
        )

    def _on_blocked(self, msg: String) -> None:
        self._print_digest(TOPIC_BLOCKED, msg.data)

    def _on_degraded(self, msg: String) -> None:
        self._print_digest(TOPIC_DEGRADED, msg.data)


def main() -> int:
    rclpy.init(args=sys.argv)
    node = P3DualAdvisoryVisibilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
