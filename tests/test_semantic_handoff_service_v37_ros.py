"""
V3.7 — automated ROS integration: ``ProduceSemanticBlockedPassageV35`` service (bounded cases).

Requires a built workspace with ``multi_robot_mission_stack_interfaces`` installed and
``install/setup.bash`` sourced so generated srv Python is importable.

Skip if ``rclpy`` or generated srv imports fail (e.g. pure PYTHONPATH-only pytest).
"""

from __future__ import annotations

import json
import uuid

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import MultiThreadedExecutor

from multi_robot_mission_stack.semantic.blocked_passage_v301 import BlockedPassageBeliefStore
from multi_robot_mission_stack.semantic.semantic_handoff_core_v35 import OUTCOME_HANDOFF_REQUEST_INVALID
from multi_robot_mission_stack.semantic.semantic_production_ingest_v34 import OUTCOME_INGEST_STORED

try:
    from multi_robot_mission_stack.demo.semantic_production_handoff_node_v35 import (
        SemanticProductionHandoffNodeV35,
    )
    from multi_robot_mission_stack_interfaces.srv import ProduceSemanticBlockedPassageV35
except ImportError:
    SemanticProductionHandoffNodeV35 = None  # type: ignore[misc,assignment]
    ProduceSemanticBlockedPassageV35 = None  # type: ignore[misc,assignment]

_SKIP_NO_ROS_INTEGRATION = pytest.mark.skipif(
    SemanticProductionHandoffNodeV35 is None or ProduceSemanticBlockedPassageV35 is None,
    reason=(
        "Handoff node or generated srv not importable — "
        "colcon build multi_robot_mission_stack_interfaces multi_robot_mission_stack && "
        "source install/setup.bash"
    ),
)


def _llm_context():
    return {
        "schema_version": "v3.3.llm_context.1",
        "location_ref": "base",
        "source_robot_id": "robot1",
        "nav_goal_status": "active",
        "stall_duration_sec": 1.0,
        "planner_status": "computing",
        "lidar_occlusion_proxy": True,
        "operator_hint": "",
    }


def _envelope_valid():
    return {
        "llm_context": _llm_context(),
        "assembly_timestamp_utc_iso": "2026-04-02T18:30:00.000Z",
        "ingest_now_utc_iso": "2026-04-02T18:30:30.000Z",
        "use_deterministic_fake_adapter": True,
    }


def _call_service(*, srv_node: "SemanticProductionHandoffNodeV35", inner: dict) -> dict:
    cli_node = rclpy.create_node("semantic_handoff_test_client_" + uuid.uuid4().hex[:10])
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(srv_node)
    ex.add_node(cli_node)
    try:
        client = cli_node.create_client(
            ProduceSemanticBlockedPassageV35,
            "/produce_semantic_blocked_passage_v35",
        )
        assert client.wait_for_service(timeout_sec=15.0)
        req = ProduceSemanticBlockedPassageV35.Request()
        req.json_request = json.dumps(inner, separators=(",", ":"))
        future = client.call_async(req)
        while not future.done():
            ex.spin_once(timeout_sec=0.2)
        assert future.done()
        resp = future.result()
        assert resp is not None
        return json.loads(resp.json_response)
    finally:
        ex.remove_node(srv_node)
        ex.remove_node(cli_node)
        cli_node.destroy_node()


@pytest.fixture
def ros_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@_SKIP_NO_ROS_INTEGRATION
def test_service_valid_fake_adapter_ingest_stored(ros_init):
    srv_node = SemanticProductionHandoffNodeV35()
    try:
        out = _call_service(srv_node=srv_node, inner=_envelope_valid())
        assert out["outcome"] == OUTCOME_INGEST_STORED
        assert "belief_id" in out
    finally:
        srv_node.destroy_node()


@_SKIP_NO_ROS_INTEGRATION
def test_service_invalid_envelope_handoff_request_invalid(ros_init):
    srv_node = SemanticProductionHandoffNodeV35()
    try:
        bad = {**_envelope_valid(), "extra_key": 1}
        out = _call_service(srv_node=srv_node, inner=bad)
        assert out["outcome"] == OUTCOME_HANDOFF_REQUEST_INVALID
        assert "detail" in out
    finally:
        srv_node.destroy_node()


@_SKIP_NO_ROS_INTEGRATION
def test_service_allowlist_reject(ros_init):
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"other_robot"}))
    srv_node = SemanticProductionHandoffNodeV35(store=store)
    try:
        out = _call_service(srv_node=srv_node, inner=_envelope_valid())
        assert out["outcome"] == "ingest_rejected"
        assert any("allowlist" in e.lower() for e in out.get("ingest_errors", []))
    finally:
        srv_node.destroy_node()


@_SKIP_NO_ROS_INTEGRATION
def test_service_ttl_at_ingest_reject(ros_init):
    srv_node = SemanticProductionHandoffNodeV35()
    try:
        inner = {
            **_envelope_valid(),
            "ingest_now_utc_iso": "2026-04-02T22:00:00.000Z",
        }
        out = _call_service(srv_node=srv_node, inner=inner)
        assert out["outcome"] == "ingest_rejected"
        assert any("expired" in e.lower() for e in out.get("ingest_errors", []))
    finally:
        srv_node.destroy_node()
