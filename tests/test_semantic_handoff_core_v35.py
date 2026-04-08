"""V3.5 handoff core — JSON envelope + V3.4 path (no ROS / no generated srv required)."""

from __future__ import annotations

import json
from datetime import datetime, timezone

import pytest

from multi_robot_mission_stack.semantic.blocked_passage_v301 import BlockedPassageBeliefStore
from multi_robot_mission_stack.semantic.semantic_handoff_core_v35 import (
    OUTCOME_HANDOFF_REQUEST_INVALID,
    execute_semantic_production_handoff_v35,
    parse_handoff_request_json,
    run_handoff_from_json_request,
)
from multi_robot_mission_stack.semantic.semantic_production_ingest_v34 import OUTCOME_INGEST_STORED


def _ctx():
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


def _envelope(*, use_fake: bool = True):
    return {
        "llm_context": _ctx(),
        "assembly_timestamp_utc_iso": "2026-04-02T18:30:00.000Z",
        "ingest_now_utc_iso": "2026-04-02T18:30:30.000Z",
        "use_deterministic_fake_adapter": use_fake,
    }


def test_parse_handoff_request_ok():
    raw = json.dumps(_envelope())
    ctx, ts_a, ts_i, fake = parse_handoff_request_json(raw)
    assert fake is True
    assert ctx["location_ref"] == "base"
    assert ts_a == datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    assert ts_i == datetime(2026, 4, 2, 18, 30, 30, tzinfo=timezone.utc)


def test_parse_rejects_unknown_key():
    env = _envelope()
    env["extra"] = 1
    with pytest.raises(ValueError, match="unknown request keys"):
        parse_handoff_request_json(json.dumps(env))


def test_parse_rejects_bad_json():
    with pytest.raises(ValueError, match="invalid JSON"):
        parse_handoff_request_json("{")


def test_execute_fake_adapter_ingest_stored():
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    r = execute_semantic_production_handoff_v35(
        llm_context=_ctx(),
        assembly_timestamp_utc=datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc),
        now_utc_ingest=datetime(2026, 4, 2, 18, 30, 30, tzinfo=timezone.utc),
        store=store,
        use_deterministic_fake_adapter=True,
    )
    assert r.outcome == OUTCOME_INGEST_STORED
    assert len(store) == 1


def test_run_handoff_from_json_roundtrip():
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    out = run_handoff_from_json_request(json.dumps(_envelope()), store=store)
    assert out["outcome"] == OUTCOME_INGEST_STORED
    assert "belief_id" in out
    assert len(store) == 1


def test_run_handoff_invalid_returns_handoff_request_invalid():
    store = BlockedPassageBeliefStore()
    out = run_handoff_from_json_request("{", store=store)
    assert out["outcome"] == OUTCOME_HANDOFF_REQUEST_INVALID
    assert "detail" in out
