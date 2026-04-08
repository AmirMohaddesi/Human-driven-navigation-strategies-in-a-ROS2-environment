#!/usr/bin/env python3
"""
V3.5 — single-node ROS service façade: JSON request → frozen V3.4 path → JSON response.

Service type: ``multi_robot_mission_stack_interfaces/srv/ProduceSemanticBlockedPassageV35``.
Default service name: ``produce_semantic_blocked_passage_v35``.

Optional param ``mirror_ingested_record_to_transport`` publishes accepted records on the frozen
V3.0.1 JSON topic (default off). Otherwise one process role: handoff + ingest only.
"""

from __future__ import annotations

import json
from typing import FrozenSet, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from multi_robot_mission_stack.semantic.blocked_passage_v301 import BlockedPassageBeliefStore
from multi_robot_mission_stack.semantic.semantic_handoff_core_v35 import run_handoff_from_json_request
from multi_robot_mission_stack.semantic.semantic_production_ingest_v34 import OUTCOME_INGEST_STORED
from multi_robot_mission_stack.transport.blocked_passage_json_v301 import (
    TRANSPORT_TOPIC_DEFAULT,
    encode_blocked_passage_record_json,
)
from multi_robot_mission_stack_interfaces.srv import ProduceSemanticBlockedPassageV35


def _allowlist_from_params(values: list) -> Optional[FrozenSet[str]]:
    if not values:
        return None
    out = frozenset(str(x).strip() for x in values if str(x).strip())
    return out if out else None


class SemanticProductionHandoffNodeV35(Node):
    def __init__(self, *, store: Optional[BlockedPassageBeliefStore] = None) -> None:
        super().__init__("semantic_production_handoff_v35")
        self.declare_parameter("service_name", "produce_semantic_blocked_passage_v35")
        # Non-empty default so rclpy infers STRING_ARRAY (empty [] becomes BYTE_ARRAY and breaks YAML overrides).
        self.declare_parameter("allowed_source_robot_ids", [""])
        # V3.9+: optional mirror of accepted ingest onto frozen V3.0.1 JSON transport (separate process).
        self.declare_parameter("mirror_ingested_record_to_transport", False)
        self.declare_parameter("mirror_transport_topic", TRANSPORT_TOPIC_DEFAULT)

        svc_name = str(self.get_parameter("service_name").value or "produce_semantic_blocked_passage_v35")

        if store is not None:
            self._store = store
        else:
            raw_allow = list(self.get_parameter("allowed_source_robot_ids").value)
            allowed = _allowlist_from_params(raw_allow)
            self._store = BlockedPassageBeliefStore(allowed_source_robot_ids=allowed)

        self._mirror_transport = bool(self.get_parameter("mirror_ingested_record_to_transport").value)
        self._transport_pub = None
        if self._mirror_transport:
            ttopic = str(self.get_parameter("mirror_transport_topic").value or TRANSPORT_TOPIC_DEFAULT)
            self._transport_pub = self.create_publisher(String, ttopic, 10)
            self.get_logger().info(
                "mirror_ingested_record_to_transport enabled -> publishing to %s" % ttopic
            )

        self._srv = self.create_service(
            ProduceSemanticBlockedPassageV35,
            svc_name,
            self._handle,
        )
        self.get_logger().info("V3.5 semantic handoff service ready: %s" % svc_name)

    def _handle(
        self,
        request: ProduceSemanticBlockedPassageV35.Request,
        response: ProduceSemanticBlockedPassageV35.Response,
    ) -> ProduceSemanticBlockedPassageV35.Response:
        try:
            payload = run_handoff_from_json_request(
                request.json_request,
                store=self._store,
            )
            if (
                self._transport_pub is not None
                and payload.get("outcome") == OUTCOME_INGEST_STORED
            ):
                bid = payload.get("belief_id")
                if bid:
                    rec = self._store.get_record(str(bid))
                    if rec is not None:
                        msg = String()
                        msg.data = encode_blocked_passage_record_json(rec)
                        self._transport_pub.publish(msg)
            response.json_response = json.dumps(payload, separators=(",", ":"))
        except Exception as exc:
            self.get_logger().exception("handoff internal error: %s", exc)
            response.json_response = json.dumps(
                {
                    "outcome": "handoff_internal_error",
                    "detail": str(exc),
                },
                separators=(",", ":"),
            )
        return response


def main() -> None:
    rclpy.init()
    try:
        node = SemanticProductionHandoffNodeV35()
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
