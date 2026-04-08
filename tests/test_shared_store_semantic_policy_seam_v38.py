"""
V3.8 — one bounded shared-store / policy seam (pure Python).

Proves: frozen V3.5 JSON handoff path ingests into the same ``BlockedPassageBeliefStore``
instance that ``BlockedPassageSharedStoreRuntime`` passes to ``MissionTools``; the existing
advisory ``navigate_to_named_location`` gate observes that store. No ROS, no schema change.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone

from multi_robot_mission_stack.agent.blocked_passage_shared_runtime_v301 import (
    BlockedPassageSharedStoreRuntime,
)
from multi_robot_mission_stack.agent.mock_mission_client import MockMissionClient
from multi_robot_mission_stack.semantic.blocked_passage_v301 import (
    BLOCKED_OUTCOME_VALUE,
    BlockedPassageBeliefStore,
)
from multi_robot_mission_stack.semantic.semantic_handoff_core_v35 import (
    OUTCOME_HANDOFF_REQUEST_INVALID,
    run_handoff_from_json_request,
)
from multi_robot_mission_stack.semantic.semantic_production_ingest_v34 import (
    OUTCOME_INGEST_STORED,
)


class CountingMockClient(MockMissionClient):
    def __init__(self) -> None:
        super().__init__()
        self.navigate_to_named_location_calls = 0

    def navigate_to_named_location(self, robot_id: str, location_name: str) -> dict:
        self.navigate_to_named_location_calls += 1
        return super().navigate_to_named_location(robot_id, location_name)


def _ctx() -> dict:
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


def _envelope(*, use_fake: bool = True) -> dict:
    return {
        "llm_context": _ctx(),
        "assembly_timestamp_utc_iso": "2026-04-02T18:30:00.000Z",
        "ingest_now_utc_iso": "2026-04-02T18:30:30.000Z",
        "use_deterministic_fake_adapter": use_fake,
    }


def test_v38_handoff_ingest_then_facade_blocks_named_nav_on_shared_store() -> None:
    """Semantic handoff (fake adapter) → store; same store → advisory block at tools/facade."""
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    handoff_out = run_handoff_from_json_request(
        json.dumps(_envelope()),
        store=store,
    )
    assert handoff_out["outcome"] == OUTCOME_INGEST_STORED
    assert len(store) == 1

    client = CountingMockClient()
    runtime = BlockedPassageSharedStoreRuntime(client, store=store)
    try:
        t_nav = datetime(2026, 4, 2, 18, 30, 30, tzinfo=timezone.utc)
        out = runtime.facade.handle_command(
            {
                "type": "navigate",
                "target": "named_location",
                "robot_id": "robot1",
                "location_name": "base",
            },
            now_utc=t_nav,
        )
        assert client.navigate_to_named_location_calls == 0
        assert out["status"] == "failed"
        assert out["goal_id"] is None
        assert out["outcome"] == BLOCKED_OUTCOME_VALUE
    finally:
        runtime.facade.close()


def test_v38_invalid_handoff_leaves_store_empty_facade_dispatches_nav() -> None:
    """Control: no successful ingest → policy does not block; client still receives dispatch."""
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    handoff_out = run_handoff_from_json_request("{", store=store)
    assert handoff_out["outcome"] == OUTCOME_HANDOFF_REQUEST_INVALID
    assert len(store) == 0

    client = CountingMockClient()
    runtime = BlockedPassageSharedStoreRuntime(client, store=store)
    try:
        t_nav = datetime(2026, 4, 2, 18, 30, 30, tzinfo=timezone.utc)
        out = runtime.facade.handle_command(
            {
                "type": "navigate",
                "target": "named_location",
                "robot_id": "robot1",
                "location_name": "base",
            },
            now_utc=t_nav,
        )
        assert client.navigate_to_named_location_calls == 1
        assert out["status"] == "accepted"
        assert out.get("goal_id")
    finally:
        runtime.facade.close()
