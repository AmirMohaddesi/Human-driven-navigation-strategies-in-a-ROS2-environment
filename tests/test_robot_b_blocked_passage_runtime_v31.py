"""V3.1 robot-B demo runtime: single store, ingest path, facade policy (deterministic)."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

import pytest

from multi_robot_mission_stack.agent.mock_mission_client import MockMissionClient
from multi_robot_mission_stack.demo.robot_b_blocked_passage_runtime_v31 import (
    RobotBBlockedPassageDemoRuntime,
)
from multi_robot_mission_stack.semantic.blocked_passage_local_stub_v301 import (
    build_blocked_passage_record_stub,
)
from multi_robot_mission_stack.semantic.blocked_passage_v301 import (
    BLOCKED_OUTCOME_VALUE,
    TTL_SKEW_ALLOWANCE_SEC,
)
from multi_robot_mission_stack.transport.blocked_passage_json_v301 import (
    encode_blocked_passage_record_json,
)


def _t0() -> datetime:
    return datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)


def _uuid4() -> str:
    return "a1b2c3d4-e5f6-4789-a012-3456789abcde"


def _obs() -> str:
    return "f47ac10b-58cc-4372-a567-0e02b2c3d479"


def _make_runtime() -> RobotBBlockedPassageDemoRuntime:
    return RobotBBlockedPassageDemoRuntime(
        MockMissionClient(),
        allowed_source_robot_ids=frozenset({"robot_a"}),
    )


def test_demo_runtime_owns_single_shared_store_visible_from_core_and_receiver() -> None:
    rclpy = pytest.importorskip("rclpy")
    rclpy.init()
    try:
        rt = _make_runtime()
        assert rt.store is rt.core.store
        node = rt.create_transport_receiver_node()
        try:
            assert node._store is rt.store
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_ingest_via_runtime_updates_shared_store() -> None:
    rt = _make_runtime()
    t0 = _t0()
    rec = build_blocked_passage_record_stub(
        belief_id=_uuid4(),
        source_robot_id="robot_a",
        location_ref="base",
        confidence=0.85,
        timestamp_utc=t0,
        ttl_sec=120.0,
        sensor_class="demo_test",
        observation_id=_obs(),
    )
    wire = encode_blocked_passage_record_json(rec)
    assert rt.ingest_transport_payload(wire, now_utc=t0).stored
    assert len(rt.store) == 1


def test_facade_built_from_runtime_sees_same_store_blocks_named_nav() -> None:
    rt = _make_runtime()
    t0 = _t0()
    wire = encode_blocked_passage_record_json(
        build_blocked_passage_record_stub(
            belief_id=_uuid4(),
            source_robot_id="robot_a",
            location_ref="base",
            confidence=0.85,
            timestamp_utc=t0,
            ttl_sec=120.0,
            sensor_class="demo_test",
            observation_id=_obs(),
        )
    )
    assert rt.ingest_transport_payload(wire, now_utc=t0).stored
    out = rt.facade.handle_command(
        {
            "type": "navigate",
            "target": "named_location",
            "robot_id": "robot1",
            "location_name": "base",
        },
        now_utc=t0,
    )
    assert out["outcome"] == BLOCKED_OUTCOME_VALUE


def test_expired_belief_does_not_block_facade_path() -> None:
    rt = _make_runtime()
    t0 = _t0()
    wire = encode_blocked_passage_record_json(
        build_blocked_passage_record_stub(
            belief_id=_uuid4(),
            source_robot_id="robot_a",
            location_ref="base",
            confidence=0.85,
            timestamp_utc=t0,
            ttl_sec=60.0,
            sensor_class="demo_test",
            observation_id=_obs(),
        )
    )
    assert rt.ingest_transport_payload(wire, now_utc=t0).stored
    late = t0 + timedelta(seconds=61 + TTL_SKEW_ALLOWANCE_SEC + 0.1)
    out = rt.facade.handle_command(
        {
            "type": "navigate",
            "target": "named_location",
            "robot_id": "robot1",
            "location_name": "base",
        },
        now_utc=late,
    )
    assert out["status"] == "accepted"
    assert out.get("goal_id")


def test_duplicate_wire_via_runtime_same_store_behavior() -> None:
    rt = _make_runtime()
    t0 = _t0()
    wire = encode_blocked_passage_record_json(
        build_blocked_passage_record_stub(
            belief_id=_uuid4(),
            source_robot_id="robot_a",
            location_ref="base",
            confidence=0.85,
            timestamp_utc=t0,
            ttl_sec=120.0,
            sensor_class="demo_test",
            observation_id=_obs(),
        )
    )
    assert rt.ingest_transport_payload(wire, now_utc=t0).stored
    assert rt.ingest_transport_payload(wire, now_utc=t0).duplicate_ignored
    assert len(rt.store) == 1
    q = rt.store.has_active_blocked_passage("base", now_utc=t0)
    assert q.has_active and len(q.active_belief_ids) == 1
