"""End-to-end tests for ``MissionAgentFacade`` with mock stack."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
from unittest import mock

import pytest

from multi_robot_mission_stack.agent.command_adapter import CommandAdapter
from multi_robot_mission_stack.agent.mission_agent_facade import MissionAgentFacade
from multi_robot_mission_stack.agent.mission_graph import MissionGraph
from multi_robot_mission_stack.agent.mission_policy import MissionPolicy
from multi_robot_mission_stack.agent.mission_tools import MissionTools
from multi_robot_mission_stack.agent.mock_mission_client import MockMissionClient
from multi_robot_mission_stack.agent.policy_config import (
    MissionPolicyConfig,
    build_default_policy_config,
)
from multi_robot_mission_stack.semantic.blocked_passage_local_stub_v301 import (
    build_blocked_passage_record_stub,
)
from multi_robot_mission_stack.semantic.blocked_passage_v301 import (
    BLOCKED_OUTCOME_VALUE,
    SCHEMA_VERSION,
    TTL_SKEW_ALLOWANCE_SEC,
    BlockedPassageBeliefStore,
)


def _uuid4() -> str:
    return "a1b2c3d4-e5f6-4789-a012-3456789abcde"


def _observation_uuid4() -> str:
    return "f47ac10b-58cc-4372-a567-0e02b2c3d479"


def _valid_blocked_record(
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
            "observation_id": _observation_uuid4(),
        },
        "ttl_sec": ttl,
        "verification_status": "unverified",
    }


def _facade_with_blocked_store(store: BlockedPassageBeliefStore) -> MissionAgentFacade:
    client = MockMissionClient()
    tools = MissionTools(client, blocked_passage_store=store)
    graph = MissionGraph(tools, policy=MissionPolicy(build_default_policy_config()))
    return MissionAgentFacade(CommandAdapter(), graph, mission_client=client)


def test_successful_navigate_pose_external_command() -> None:
    facade = MissionAgentFacade.with_mock()
    try:
        out = facade.handle_command(
            {
                "type": "navigate",
                "target": "pose",
                "robot_id": "robot1",
                "x": 1.0,
                "y": 2.0,
                "yaw": 0.0,
            }
        )
        assert out["status"] == "accepted"
        assert out["nav_status"] == "submitted"
        assert out.get("goal_id")
    finally:
        facade.close()


def test_successful_navigate_external_command() -> None:
    facade = MissionAgentFacade.with_mock()
    try:
        out = facade.handle_command(
            {
                "type": "navigate",
                "target": "named_location",
                "robot_id": "robot1",
                "location_name": "base",
            }
        )
        assert out["status"] == "accepted"
        assert out["nav_status"] == "submitted"
        assert out.get("goal_id")
    finally:
        facade.close()


def test_successful_state_query_after_navigation() -> None:
    facade = MissionAgentFacade.with_mock()
    try:
        nav = facade.handle_command(
            {
                "type": "navigate",
                "target": "named_location",
                "robot_id": "robot2",
                "location_name": "test_goal",
            }
        )
        gid = nav["goal_id"]
        state = facade.handle_command(
            {
                "type": "query",
                "target": "navigation_state",
                "robot_id": "robot2",
                "goal_id": gid,
            }
        )
        assert state["status"] == "success"
        assert state["nav_status"] == "in_progress"
    finally:
        facade.close()


def test_invalid_external_command_rejected_before_graph() -> None:
    facade = MissionAgentFacade.with_mock()
    try:
        out = facade.handle_command(
            {
                "type": "navigate",
                "target": "bad_target",
                "robot_id": "robot1",
                "location_name": "base",
            }
        )
        assert out["status"] == "failed"
        assert out["nav_status"] == "unknown"
        assert "named_location" in out["message"]
    finally:
        facade.close()


def test_policy_denial_through_facade_with_custom_policy() -> None:
    cfg = MissionPolicyConfig(
        allowed_actions=frozenset(
            {
                "navigate_to_named_location",
                "navigate_to_pose",
                "get_navigation_state",
                "cancel_navigation",
            }
        ),
        allowed_robot_ids=frozenset({"robot1", "robot2"}),
        allowed_locations=frozenset({"test_goal"}),
    )
    facade = MissionAgentFacade.with_mock(policy=MissionPolicy(cfg))
    try:
        out = facade.handle_command(
            {
                "type": "navigate",
                "target": "named_location",
                "robot_id": "robot1",
                "location_name": "base",
            }
        )
        assert out["status"] == "failed"
        assert out["nav_status"] == "unknown"
        assert "policy denied" in out["message"].lower()
        assert "base" in out["message"]
    finally:
        facade.close()


def test_close_is_safe_to_call() -> None:
    facade = MissionAgentFacade.with_mock()
    facade.close()
    facade.close()


def test_cancel_navigation_after_navigate() -> None:
    facade = MissionAgentFacade.with_mock()
    try:
        nav = facade.handle_command(
            {
                "type": "navigate",
                "target": "named_location",
                "robot_id": "robot1",
                "location_name": "base",
            }
        )
        gid = nav["goal_id"]
        out = facade.handle_command(
            {
                "type": "cancel",
                "target": "navigation",
                "robot_id": "robot1",
                "goal_id": str(gid),
            }
        )
        assert out["status"] == "success"
        assert out["nav_status"] == "canceled"
    finally:
        facade.close()


def test_cancel_unknown_goal_id_structured_failure() -> None:
    facade = MissionAgentFacade.with_mock()
    try:
        out = facade.handle_command(
            {
                "type": "cancel",
                "target": "navigation",
                "robot_id": "robot1",
                "goal_id": "nonexistent-goal-id",
            }
        )
        assert out["status"] == "failure"
        assert out["nav_status"] == "unknown"
    finally:
        facade.close()


def test_handle_command_forwards_now_utc_to_graph_invoke() -> None:
    facade = MissionAgentFacade.with_mock()
    t0 = datetime(2026, 1, 1, 0, 0, 0, tzinfo=timezone.utc)
    try:
        real_invoke = facade._graph.invoke
        with mock.patch.object(
            facade._graph, "invoke", wraps=real_invoke
        ) as wrapped:
            facade.handle_command(
                {
                    "type": "navigate",
                    "target": "named_location",
                    "robot_id": "robot1",
                    "location_name": "base",
                },
                now_utc=t0,
            )
            wrapped.assert_called_once()
            assert wrapped.call_args.kwargs.get("now_utc") is t0
    finally:
        facade.close()


def test_facade_blocked_passage_active_blocks_named_nav() -> None:
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    assert store.ingest(_valid_blocked_record(location="base"), now_utc=t0).stored
    facade = _facade_with_blocked_store(store)
    try:
        out = facade.handle_command(
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
    finally:
        facade.close()


def test_facade_blocked_passage_expired_does_not_block() -> None:
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    rec = _valid_blocked_record(ts="2026-04-02T18:30:00.000Z", ttl=60.0)
    assert store.ingest(rec, now_utc=t0).stored
    late = t0 + timedelta(seconds=61 + TTL_SKEW_ALLOWANCE_SEC + 0.1)
    facade = _facade_with_blocked_store(store)
    try:
        out = facade.handle_command(
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
    finally:
        facade.close()


def test_facade_blocked_passage_empty_store_navigates_with_now_utc() -> None:
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    store = BlockedPassageBeliefStore()
    facade = _facade_with_blocked_store(store)
    try:
        out = facade.handle_command(
            {
                "type": "navigate",
                "target": "named_location",
                "robot_id": "robot1",
                "location_name": "base",
            },
            now_utc=t0,
        )
        assert out["status"] == "accepted"
        assert out.get("goal_id")
    finally:
        facade.close()


def test_facade_blocked_store_without_now_utc_fails_clearly() -> None:
    store = BlockedPassageBeliefStore()
    facade = _facade_with_blocked_store(store)
    try:
        out = facade.handle_command(
            {
                "type": "navigate",
                "target": "named_location",
                "robot_id": "robot1",
                "location_name": "base",
            }
        )
        assert out["status"] == "failed"
        assert "now_utc" in out["message"].lower()
    finally:
        facade.close()


def test_facade_local_stub_build_ingest_blocks_named_nav() -> None:
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    rec = build_blocked_passage_record_stub(
        belief_id=_uuid4(),
        source_robot_id="robot1",
        location_ref="base",
        confidence=0.85,
        timestamp_utc=t0,
        ttl_sec=120.0,
        sensor_class="local_stub",
        observation_id=_observation_uuid4(),
    )
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    assert store.ingest(rec, now_utc=t0).stored
    facade = _facade_with_blocked_store(store)
    try:
        out = facade.handle_command(
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
    finally:
        facade.close()


def test_facade_local_stub_expired_belief_navigates() -> None:
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    rec = build_blocked_passage_record_stub(
        belief_id=_uuid4(),
        source_robot_id="robot1",
        location_ref="base",
        confidence=0.85,
        timestamp_utc=t0,
        ttl_sec=60.0,
        sensor_class="local_stub",
        observation_id=_observation_uuid4(),
    )
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    assert store.ingest(rec, now_utc=t0).stored
    late = t0 + timedelta(seconds=61 + TTL_SKEW_ALLOWANCE_SEC + 0.1)
    facade = _facade_with_blocked_store(store)
    try:
        out = facade.handle_command(
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
    finally:
        facade.close()
