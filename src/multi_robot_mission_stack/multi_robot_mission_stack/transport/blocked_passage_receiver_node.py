#!/usr/bin/env python3
"""ROS 2 subscriber: JSON ``std_msgs/String`` -> ``BlockedPassageBeliefStore.ingest`` (V3.0.1).

Pass ``store=`` to share one ``BlockedPassageBeliefStore`` with
``BlockedPassageSharedStoreRuntime`` (same ingest path as ``ingest_transport_payload``).
"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import FrozenSet, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .blocked_passage_json_v301 import (
    TRANSPORT_TOPIC_DEFAULT,
    ingest_blocked_passage_transport_payload,
)
from ..semantic.blocked_passage_v301 import BlockedPassageBeliefStore


def _allowlist_from_params(values: list) -> Optional[FrozenSet[str]]:
    if not values:
        return None
    out = frozenset(str(x).strip() for x in values if str(x).strip())
    return out if out else None


class BlockedPassageTransportReceiverNode(Node):
    def __init__(self, *, store: Optional[BlockedPassageBeliefStore] = None) -> None:
        super().__init__("blocked_passage_transport_receiver")
        self.declare_parameter("topic", TRANSPORT_TOPIC_DEFAULT)
        self.declare_parameter("allowed_source_robot_ids", [])

        topic = str(self.get_parameter("topic").value or TRANSPORT_TOPIC_DEFAULT)

        if store is not None:
            self._store = store
        else:
            raw_allow = list(self.get_parameter("allowed_source_robot_ids").value)
            allowed = _allowlist_from_params(raw_allow)
            self._store = BlockedPassageBeliefStore(allowed_source_robot_ids=allowed)

        qos = 10
        self.create_subscription(String, topic, self._on_string, qos)
        self.get_logger().info("subscribed to %s (ingest uses ROS clock at receive time)" % topic)

    def _on_string(self, msg: String) -> None:
        now = self.get_clock().now()
        ns = int(now.nanoseconds)
        now_utc = datetime.fromtimestamp(ns / 1e9, tz=timezone.utc)
        res = ingest_blocked_passage_transport_payload(self._store, msg.data, now_utc=now_utc)
        if res.stored:
            self.get_logger().info("ingested blocked_passage (belief stored)")
        elif res.duplicate_ignored:
            self.get_logger().info("duplicate belief_id ignored")
        elif res.rejected:
            self.get_logger().warning("ingest rejected: %s" % ("; ".join(res.errors),))


def main() -> None:
    rclpy.init()
    try:
        node = BlockedPassageTransportReceiverNode()
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
