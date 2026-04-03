"""V3.0.1 single-hop transport: JSON wire format + shared-store policy (no live ROS required)."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

import pytest

from multi_robot_mission_stack.agent.mission_tools import MissionTools
from multi_robot_mission_stack.agent.mock_mission_client import MockMissionClient
from multi_robot_mission_stack.semantic.blocked_passage_local_stub_v301 import (
    build_blocked_passage_record_stub,
)
from multi_robot_mission_stack.semantic.blocked_passage_v301 import (
    BLOCKED_OUTCOME_VALUE,
    TTL_SKEW_ALLOWANCE_SEC,
    BlockedPassageBeliefStore,
    validate_blocked_passage_record,
)
from multi_robot_mission_stack.transport.blocked_passage_json_v301 import (
    decode_blocked_passage_transport_payload,
    encode_blocked_passage_record_json,
    ingest_blocked_passage_transport_payload,
)


def _uuid4() -> str:
    return "a1b2c3d4-e5f6-4789-a012-3456789abcde"


def _obs_id() -> str:
    return "f47ac10b-58cc-4372-a567-0e02b2c3d479"


def _t0() -> datetime:
    return datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)


def _valid_record_dict(t0: datetime | None = None) -> dict:
    ts = t0 or _t0()
    return build_blocked_passage_record_stub(
        belief_id=_uuid4(),
        source_robot_id="robot_a",
        location_ref="base",
        confidence=0.85,
        timestamp_utc=ts,
        ttl_sec=120.0,
        sensor_class="transport_test",
        observation_id=_obs_id(),
    )


def test_encode_decode_roundtrip_passes_schema() -> None:
    rec = _valid_record_dict()
    wire = encode_blocked_passage_record_json(rec)
    got = decode_blocked_passage_transport_payload(wire)
    ok, errs = validate_blocked_passage_record(got)
    assert ok and errs == []
    assert got["belief_id"] == rec["belief_id"]


def test_ingest_transport_payload_stores_fresh_fact() -> None:
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot_a"}))
    t0 = _t0()
    wire = encode_blocked_passage_record_json(_valid_record_dict(t0))
    res = ingest_blocked_passage_transport_payload(store, wire, now_utc=t0)
    assert res.stored and not res.rejected
    assert len(store) == 1


def test_transport_then_tools_blocks_matching_named_location() -> None:
    t0 = _t0()
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot_a"}))
    wire = encode_blocked_passage_record_json(_valid_record_dict(t0))
    assert ingest_blocked_passage_transport_payload(store, wire, now_utc=t0).stored
    client = MockMissionClient()
    tools = MissionTools(client, blocked_passage_store=store)
    out = tools.navigate_to_named_location("robot1", "base", now_utc=t0)
    assert out["outcome"] == BLOCKED_OUTCOME_VALUE


def test_duplicate_transport_payload_does_not_add_second_belief() -> None:
    t0 = _t0()
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot_a"}))
    wire = encode_blocked_passage_record_json(_valid_record_dict(t0))
    assert ingest_blocked_passage_transport_payload(store, wire, now_utc=t0).stored
    res2 = ingest_blocked_passage_transport_payload(store, wire, now_utc=t0)
    assert res2.duplicate_ignored and not res2.stored
    assert len(store) == 1
    q = store.has_active_blocked_passage("base", now_utc=t0)
    assert q.has_active and len(q.active_belief_ids) == 1


def test_expired_fact_from_transport_does_not_block_later_nav() -> None:
    t0 = _t0()
    rec = build_blocked_passage_record_stub(
        belief_id=_uuid4(),
        source_robot_id="robot_a",
        location_ref="base",
        confidence=0.85,
        timestamp_utc=t0,
        ttl_sec=60.0,
        sensor_class="transport_test",
        observation_id=_obs_id(),
    )
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot_a"}))
    wire = encode_blocked_passage_record_json(rec)
    assert ingest_blocked_passage_transport_payload(store, wire, now_utc=t0).stored
    late = t0 + timedelta(seconds=61 + TTL_SKEW_ALLOWANCE_SEC + 0.1)
    client = MockMissionClient()
    tools = MissionTools(client, blocked_passage_store=store)
    out = tools.navigate_to_named_location("robot1", "base", now_utc=late)
    assert out["status"] == "accepted"
    assert out.get("goal_id")


def test_invalid_json_payload_raises() -> None:
    store = BlockedPassageBeliefStore()
    with pytest.raises(ValueError, match="JSON"):
        ingest_blocked_passage_transport_payload(store, "not json {{{", now_utc=_t0())


def test_non_object_json_raises() -> None:
    with pytest.raises(ValueError, match="object"):
        decode_blocked_passage_transport_payload("[1,2,3]")
