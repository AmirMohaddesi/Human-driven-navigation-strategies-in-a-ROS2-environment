"""Unit tests for V3.3 LLM path → final blocked_passage assembly (no ROS)."""

from __future__ import annotations

import json
import uuid
from datetime import datetime, timezone

import pytest

from multi_robot_mission_stack.semantic.blocked_passage_v301 import validate_blocked_passage_record
from multi_robot_mission_stack.semantic.llm_blocked_passage_assembler_v33 import (
    FINAL_SCHEMA_REJECT,
    assemble_blocked_passage_from_v33_accept,
)
from multi_robot_mission_stack.semantic.llm_candidate_boundary_v33 import evaluate_llm_candidate_boundary

_FIXED_TS = datetime(2024, 6, 1, 12, 0, 0, tzinfo=timezone.utc)
_KNOWN_BELIEF = "12345678-1234-4678-8234-567812345678"
_KNOWN_OBS = "87654321-4321-4678-8234-567812345678"


def _accept():
    ctx = {
        "schema_version": "v3.3.llm_context.1",
        "location_ref": "base",
        "source_robot_id": "r1",
        "nav_goal_status": "active",
        "stall_duration_sec": 12.0,
        "planner_status": "computing",
        "lidar_occlusion_proxy": True,
        "operator_hint": "",
    }
    out = {
        "schema_version": "v3.3.llm_candidate.1",
        "assert_blocked": True,
        "location_ref": "base",
        "confidence": 0.7,
        "ttl_sec": 3600.0,
        "sensor_class": "lidar_occlusion",
    }
    r = evaluate_llm_candidate_boundary(ctx, json.dumps(out))
    assert r.accepted is True
    return r


def test_assembler_happy_path_passes_final_validation():
    accept = _accept()
    result = assemble_blocked_passage_from_v33_accept(
        accept,
        timestamp_utc=_FIXED_TS,
        belief_id=_KNOWN_BELIEF,
        observation_id=_KNOWN_OBS,
    )
    assert result.accepted is True
    rec = result.record
    ok, errs = validate_blocked_passage_record(rec)
    assert ok and errs == []
    assert rec["schema_version"] == "v3.0.1"
    assert rec["fact_type"] == "blocked_passage"
    assert rec["belief_id"] == _KNOWN_BELIEF
    assert rec["source_robot_id"] == "r1"
    assert rec["timestamp_utc"] == "2024-06-01T12:00:00Z"
    assert rec["confidence"] == 0.7
    assert rec["location_ref"] == "base"
    assert rec["provenance"]["sensor_class"] == "lidar_occlusion"
    assert rec["provenance"]["observation_id"] == _KNOWN_OBS
    assert rec["ttl_sec"] == 3600.0
    assert rec["verification_status"] == "unverified"
    assert "rationale_short" not in rec


def test_assembler_generates_uuids_when_omitted():
    accept = _accept()
    result = assemble_blocked_passage_from_v33_accept(accept, timestamp_utc=_FIXED_TS)
    assert result.accepted is True
    bid = result.record["belief_id"]
    oid = result.record["provenance"]["observation_id"]
    uuid.UUID(bid, version=4)
    uuid.UUID(oid, version=4)
    ok, _ = validate_blocked_passage_record(result.record)
    assert ok


def test_final_schema_reject_malformed_belief_id():
    accept = _accept()
    result = assemble_blocked_passage_from_v33_accept(
        accept,
        timestamp_utc=_FIXED_TS,
        belief_id="not-a-uuid-v4",
        observation_id=_KNOWN_OBS,
    )
    assert result.accepted is False
    assert result.reason == FINAL_SCHEMA_REJECT
    assert "belief_id" in result.detail.lower()


def test_final_schema_reject_malformed_observation_id():
    accept = _accept()
    result = assemble_blocked_passage_from_v33_accept(
        accept,
        timestamp_utc=_FIXED_TS,
        belief_id=_KNOWN_BELIEF,
        observation_id="bad",
    )
    assert result.accepted is False
    assert result.reason == FINAL_SCHEMA_REJECT
    assert "observation_id" in result.detail.lower()


def test_timestamp_naive_raises():
    accept = _accept()
    naive = datetime(2024, 6, 1, 12, 0, 0)
    with pytest.raises(ValueError, match="timezone-aware"):
        assemble_blocked_passage_from_v33_accept(accept, timestamp_utc=naive)
