#!/usr/bin/env python3
"""
V3.9 — second process role: ingest frozen V3.0.1 transport JSON into a local store shared with
``BlockedPassageSharedStoreRuntime``, then expose a narrow service to observe the existing advisory
named-location outcome (no new policy semantics).
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from typing import FrozenSet, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from multi_robot_mission_stack.agent.blocked_passage_shared_runtime_v301 import (
    BlockedPassageSharedStoreRuntime,
)
from multi_robot_mission_stack.agent.mock_mission_client import MockMissionClient
from multi_robot_mission_stack.semantic.blocked_passage_v301 import BlockedPassageBeliefStore
from multi_robot_mission_stack.transport.blocked_passage_json_v301 import (
    TRANSPORT_TOPIC_DEFAULT,
    ingest_blocked_passage_transport_payload,
)
from multi_robot_mission_stack_interfaces.srv import QueryAdvisoryNamedNavV39


def _allowlist_from_params(values: list) -> Optional[FrozenSet[str]]:
    if not values:
        return None
    out = frozenset(str(x).strip() for x in values if str(x).strip())
    return out if out else None


def _parse_iso_utc(s: str) -> datetime:
    raw = str(s).strip()
    if raw.endswith("Z"):
        raw = raw[:-1] + "+00:00"
    dt = datetime.fromisoformat(raw)
    if dt.tzinfo is None:
        raise ValueError("decision_now_utc_iso must be timezone-aware (RFC 3339)")
    return dt.astimezone(timezone.utc)


class AdvisorySeamWitnessNodeV39(Node):
    def __init__(self) -> None:
        super().__init__("advisory_seam_witness_v39")
        self.declare_parameter("transport_topic", TRANSPORT_TOPIC_DEFAULT)
        self.declare_parameter("allowed_source_robot_ids", [""])
        self.declare_parameter("service_name", "query_advisory_named_nav_v39")

        topic = str(self.get_parameter("transport_topic").value or TRANSPORT_TOPIC_DEFAULT)
        svc_name = str(self.get_parameter("service_name").value or "query_advisory_named_nav_v39")
        raw_allow = list(self.get_parameter("allowed_source_robot_ids").value)
        allowed = _allowlist_from_params(raw_allow)
        self._store = BlockedPassageBeliefStore(allowed_source_robot_ids=allowed)
        client = MockMissionClient()
        self._runtime = BlockedPassageSharedStoreRuntime(client, store=self._store)

        self.create_subscription(String, topic, self._on_transport, 10)
        self.create_service(QueryAdvisoryNamedNavV39, svc_name, self._handle_query)
        self.get_logger().info(
            "V3.9 advisory witness: sub=%s svc=%s" % (topic, svc_name)
        )

    def _on_transport(self, msg: String) -> None:
        now = self.get_clock().now()
        ns = int(now.nanoseconds)
        now_utc = datetime.fromtimestamp(ns / 1e9, tz=timezone.utc)
        res = ingest_blocked_passage_transport_payload(self._store, msg.data, now_utc=now_utc)
        if res.stored:
            self.get_logger().info("ingested transport -> local advisory store")
        elif res.duplicate_ignored:
            self.get_logger().info("duplicate belief_id ignored")
        elif res.rejected:
            self.get_logger().warning("ingest rejected: %s" % ("; ".join(res.errors),))

    def _handle_query(
        self,
        request: QueryAdvisoryNamedNavV39.Request,
        response: QueryAdvisoryNamedNavV39.Response,
    ) -> QueryAdvisoryNamedNavV39.Response:
        try:
            now_utc = _parse_iso_utc(str(request.decision_now_utc_iso))
        except ValueError as exc:
            response.ok = False
            response.json_result = json.dumps(
                {"status": "failed", "message": str(exc)},
                separators=(",", ":"),
            )
            return response
        out = self._runtime.facade.handle_command(
            {
                "type": "navigate",
                "target": "named_location",
                "robot_id": str(request.robot_id).strip(),
                "location_name": str(request.location_name).strip(),
            },
            now_utc=now_utc,
        )
        response.ok = True
        response.json_result = json.dumps(out, separators=(",", ":"), default=str)
        return response


def main() -> None:
    rclpy.init()
    try:
        node = AdvisorySeamWitnessNodeV39()
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
