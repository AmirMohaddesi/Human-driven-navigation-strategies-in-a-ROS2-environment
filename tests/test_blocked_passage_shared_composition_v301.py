"""V3.0.1 single-process shared store: transport ingest + MissionTools policy (deterministic)."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

from multi_robot_mission_stack.agent.blocked_passage_shared_runtime_v301 import (
    BlockedPassageSharedStoreRuntime,
)
from multi_robot_mission_stack.agent.mock_mission_client import MockMissionClient
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


def _obs_id() -> str:
    return "f47ac10b-58cc-4372-a567-0e02b2c3d479"


def _wire_for_base(t0: datetime | None = None, *, ttl_sec: float = 120.0) -> str:
    ts = t0 or _t0()
    rec = build_blocked_passage_record_stub(
        belief_id=_uuid4(),
        source_robot_id="robot_a",
        location_ref="base",
        confidence=0.85,
        timestamp_utc=ts,
        ttl_sec=ttl_sec,
        sensor_class="composition_test",
        observation_id=_obs_id(),
    )
    return encode_blocked_passage_record_json(rec)


def test_shared_runtime_ingest_then_facade_blocks_named_nav() -> None:
    t0 = _t0()
    client = MockMissionClient()
    rt = BlockedPassageSharedStoreRuntime(
        client,
        allowed_source_robot_ids=frozenset({"robot_a"}),
    )
    wire = _wire_for_base(t0)
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
    assert out["goal_id"] is None


def test_expired_belief_after_transport_ingest_does_not_block_facade() -> None:
    t0 = _t0()
    client = MockMissionClient()
    rt = BlockedPassageSharedStoreRuntime(
        client,
        allowed_source_robot_ids=frozenset({"robot_a"}),
    )
    wire = _wire_for_base(t0, ttl_sec=60.0)
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


def test_duplicate_wire_payload_single_active_belief_facade_still_blocked_once() -> None:
    t0 = _t0()
    client = MockMissionClient()
    rt = BlockedPassageSharedStoreRuntime(
        client,
        allowed_source_robot_ids=frozenset({"robot_a"}),
    )
    wire = _wire_for_base(t0)
    assert rt.ingest_transport_payload(wire, now_utc=t0).stored
    res2 = rt.ingest_transport_payload(wire, now_utc=t0)
    assert res2.duplicate_ignored
    assert len(rt.store) == 1
    q = rt.store.has_active_blocked_passage("base", now_utc=t0)
    assert q.has_active and len(q.active_belief_ids) == 1
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

