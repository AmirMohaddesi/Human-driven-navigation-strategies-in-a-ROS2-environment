"""V3.0.1 policy hook: ``MissionTools.navigate_to_named_location`` + optional graph ``now_utc``."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

import pytest

from multi_robot_mission_stack.agent.mission_graph import MissionGraph
from multi_robot_mission_stack.agent.mission_policy import MissionPolicy
from multi_robot_mission_stack.agent.mission_tools import MissionTools
from multi_robot_mission_stack.agent.mock_mission_client import MockMissionClient
from multi_robot_mission_stack.agent.policy_config import build_default_policy_config
from multi_robot_mission_stack.semantic.blocked_passage_local_stub_v301 import (
    build_blocked_passage_record_stub,
)
from multi_robot_mission_stack.semantic.blocked_passage_v301 import (
    BLOCKED_OUTCOME_SCHEMA_VERSION,
    BLOCKED_OUTCOME_VALUE,
    BLOCKED_REASON_CODE,
    SCHEMA_VERSION,
    TTL_SKEW_ALLOWANCE_SEC,
    BlockedPassageBeliefStore,
)


def _uuid4() -> str:
    return "a1b2c3d4-e5f6-4789-a012-3456789abcde"


def _valid_record(
    *,
    belief_id: str | None = None,
    ts: str | None = None,
    ttl: float = 120.0,
    location: str = "base",
) -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "belief_id": belief_id or _uuid4(),
        "fact_type": "blocked_passage",
        "source_robot_id": "robot1",
        "timestamp_utc": ts or "2026-04-02T18:30:00.000Z",
        "confidence": 0.85,
        "location_ref": location,
        "provenance": {
            "sensor_class": "lidar_occlusion",
            "observation_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
        },
        "ttl_sec": ttl,
        "verification_status": "unverified",
    }


class CountingMockClient(MockMissionClient):
    def __init__(self) -> None:
        super().__init__()
        self.navigate_to_named_location_calls = 0

    def navigate_to_named_location(self, robot_id: str, location_name: str) -> dict:
        self.navigate_to_named_location_calls += 1
        return super().navigate_to_named_location(robot_id, location_name)


@pytest.fixture
def t0() -> datetime:
    return datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)


def test_active_belief_blocks_dispatch_at_tools_seam(t0: datetime) -> None:
    client = CountingMockClient()
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    assert store.ingest(_valid_record(location="base"), now_utc=t0).stored
    tools = MissionTools(client, blocked_passage_store=store)
    out = tools.navigate_to_named_location("robot1", "base", now_utc=t0)
    assert client.navigate_to_named_location_calls == 0
    assert out["status"] == "failed"
    assert out["goal_id"] is None
    assert out["outcome"] == BLOCKED_OUTCOME_VALUE
    assert out["schema_version"] == BLOCKED_OUTCOME_SCHEMA_VERSION
    assert out["reason_code"] == BLOCKED_REASON_CODE
    assert out["requested_location_name"] == "base"
    assert out["active_belief_ids"] == [_uuid4()]


def test_expired_belief_does_not_block(t0: datetime) -> None:
    client = CountingMockClient()
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    rec = _valid_record(ts="2026-04-02T18:30:00.000Z", ttl=60.0)
    assert store.ingest(rec, now_utc=t0).stored
    late = t0 + timedelta(seconds=61 + TTL_SKEW_ALLOWANCE_SEC + 0.1)
    tools = MissionTools(client, blocked_passage_store=store)
    out = tools.navigate_to_named_location("robot1", "base", now_utc=late)
    assert client.navigate_to_named_location_calls == 1
    assert out["status"] == "accepted"
    assert out.get("goal_id")


def test_no_store_path_unchanged_no_now_utc_required() -> None:
    client = CountingMockClient()
    tools = MissionTools(client)
    out = tools.navigate_to_named_location("robot1", "base")
    assert client.navigate_to_named_location_calls == 1
    assert out["status"] == "accepted"


def test_store_requires_now_utc() -> None:
    client = CountingMockClient()
    store = BlockedPassageBeliefStore()
    tools = MissionTools(client, blocked_passage_store=store)
    out = tools.navigate_to_named_location("robot1", "base")
    assert client.navigate_to_named_location_calls == 0
    assert out["status"] == "failed"
    assert "now_utc" in out["message"].lower()


def test_graph_active_belief_blocks_with_invoke_now_utc(t0: datetime) -> None:
    client = CountingMockClient()
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    assert store.ingest(_valid_record(location="base"), now_utc=t0).stored
    tools = MissionTools(client, blocked_passage_store=store)
    graph = MissionGraph(tools, policy=MissionPolicy(build_default_policy_config()))
    out = graph.invoke(
        {
            "action": "navigate_to_named_location",
            "robot_id": "robot1",
            "location_name": "base",
        },
        now_utc=t0,
    )
    assert client.navigate_to_named_location_calls == 0
    assert out["outcome"] == BLOCKED_OUTCOME_VALUE
    assert out["reason_code"] == BLOCKED_REASON_CODE


def test_graph_no_belief_dispatches_normally(t0: datetime) -> None:
    client = CountingMockClient()
    store = BlockedPassageBeliefStore()
    tools = MissionTools(client, blocked_passage_store=store)
    graph = MissionGraph(tools, policy=MissionPolicy(build_default_policy_config()))
    out = graph.invoke(
        {
            "action": "navigate_to_named_location",
            "robot_id": "robot1",
            "location_name": "base",
        },
        now_utc=t0,
    )
    assert client.navigate_to_named_location_calls == 1
    assert out["status"] == "accepted"
    assert out.get("goal_id")


def test_stub_built_record_blocks_tools_named_location_path(t0: datetime) -> None:
    client = CountingMockClient()
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    rec = build_blocked_passage_record_stub(
        belief_id=_uuid4(),
        source_robot_id="robot1",
        location_ref="base",
        confidence=0.85,
        timestamp_utc=t0,
        ttl_sec=120.0,
        sensor_class="local_stub",
        observation_id="f47ac10b-58cc-4372-a567-0e02b2c3d479",
    )
    assert store.ingest(rec, now_utc=t0).stored
    tools = MissionTools(client, blocked_passage_store=store)
    out = tools.navigate_to_named_location("robot1", "base", now_utc=t0)
    assert client.navigate_to_named_location_calls == 0
    assert out["outcome"] == BLOCKED_OUTCOME_VALUE


def test_stub_built_record_expired_does_not_block_tools_path(t0: datetime) -> None:
    client = CountingMockClient()
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    rec = build_blocked_passage_record_stub(
        belief_id=_uuid4(),
        source_robot_id="robot1",
        location_ref="base",
        confidence=0.85,
        timestamp_utc=t0,
        ttl_sec=60.0,
        sensor_class="local_stub",
        observation_id="f47ac10b-58cc-4372-a567-0e02b2c3d479",
    )
    assert store.ingest(rec, now_utc=t0).stored
    late = t0 + timedelta(seconds=61 + TTL_SKEW_ALLOWANCE_SEC + 0.1)
    tools = MissionTools(client, blocked_passage_store=store)
    out = tools.navigate_to_named_location("robot1", "base", now_utc=late)
    assert client.navigate_to_named_location_calls == 1
    assert out["status"] == "accepted"
