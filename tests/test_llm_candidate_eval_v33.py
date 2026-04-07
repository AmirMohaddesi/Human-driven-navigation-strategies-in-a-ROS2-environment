"""Local V3.3 chain: context + fake model text → boundary → assembler → classified outcome."""

from __future__ import annotations

import json
from datetime import datetime, timezone

import pytest

from multi_robot_mission_stack.semantic.blocked_passage_v301 import validate_blocked_passage_record
from multi_robot_mission_stack.semantic.llm_blocked_passage_assembler_v33 import FINAL_SCHEMA_REJECT
from multi_robot_mission_stack.semantic.llm_candidate_boundary_v33 import (
    CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH,
    CANDIDATE_MALFORMED,
    CANDIDATE_SEMANTICALLY_INSUFFICIENT,
)
from multi_robot_mission_stack.semantic.llm_candidate_eval_v33 import (
    OUTCOME_ACCEPTED_RECORD,
    OUTCOME_BOUNDARY_REJECT,
    OUTCOME_FINAL_SCHEMA_REJECT,
    LlmLocalEvalResult,
    evaluate_llm_candidate_local_v33,
)

FIXED_TS = datetime(2024, 6, 1, 12, 0, 0, tzinfo=timezone.utc)
VALID_BELIEF_ID = "12345678-1234-4678-8234-567812345678"
VALID_OBS_ID = "87654321-4321-4678-8234-567812345678"


@pytest.fixture
def llm_context_base():
    return {
        "schema_version": "v3.3.llm_context.1",
        "location_ref": "base",
        "source_robot_id": "r1",
        "nav_goal_status": "active",
        "stall_duration_sec": 12.0,
        "planner_status": "computing",
        "lidar_occlusion_proxy": True,
        "operator_hint": "",
    }


@pytest.fixture
def good_candidate_dict():
    return {
        "schema_version": "v3.3.llm_candidate.1",
        "assert_blocked": True,
        "location_ref": "base",
        "confidence": 0.82,
        "ttl_sec": 180.0,
        "sensor_class": "lidar_occlusion",
    }


def test_accepted_record_full_chain(llm_context_base, good_candidate_dict):
    raw = json.dumps(good_candidate_dict)
    r = evaluate_llm_candidate_local_v33(
        llm_context_base,
        raw,
        timestamp_utc=FIXED_TS,
        belief_id=VALID_BELIEF_ID,
        observation_id=VALID_OBS_ID,
    )
    assert isinstance(r, LlmLocalEvalResult)
    assert r.outcome == OUTCOME_ACCEPTED_RECORD
    assert r.boundary_reason is None
    assert r.assembly_reason is None
    assert r.record is not None
    ok, errs = validate_blocked_passage_record(r.record)
    assert ok and errs == []
    assert r.record["belief_id"] == VALID_BELIEF_ID
    assert r.record["timestamp_utc"] == "2024-06-01T12:00:00Z"


def test_boundary_reject_malformed_json(llm_context_base):
    r = evaluate_llm_candidate_local_v33(
        llm_context_base,
        "not json",
        timestamp_utc=FIXED_TS,
    )
    assert r.outcome == OUTCOME_BOUNDARY_REJECT
    assert r.boundary_reason == CANDIDATE_MALFORMED
    assert r.record is None


def test_boundary_reject_contract_invalid_location(llm_context_base, good_candidate_dict):
    bad = {**good_candidate_dict, "location_ref": "other"}
    r = evaluate_llm_candidate_local_v33(
        llm_context_base,
        json.dumps(bad),
        timestamp_utc=FIXED_TS,
    )
    assert r.outcome == OUTCOME_BOUNDARY_REJECT
    assert r.boundary_reason == CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH
    assert r.record is None


def test_boundary_reject_candidate_not_asserted(llm_context_base):
    raw = json.dumps(
        {
            "schema_version": "v3.3.llm_candidate.1",
            "assert_blocked": False,
        }
    )
    r = evaluate_llm_candidate_local_v33(
        llm_context_base,
        raw,
        timestamp_utc=FIXED_TS,
    )
    assert r.outcome == OUTCOME_BOUNDARY_REJECT
    assert r.boundary_reason == CANDIDATE_SEMANTICALLY_INSUFFICIENT
    assert r.record is None


def test_final_schema_reject_via_injected_bad_belief_id(llm_context_base, good_candidate_dict):
    r = evaluate_llm_candidate_local_v33(
        llm_context_base,
        json.dumps(good_candidate_dict),
        timestamp_utc=FIXED_TS,
        belief_id="not-uuid-v4",
        observation_id=VALID_OBS_ID,
    )
    assert r.outcome == OUTCOME_FINAL_SCHEMA_REJECT
    assert r.assembly_reason == FINAL_SCHEMA_REJECT
    assert r.record is None
    assert "belief_id" in (r.assembly_detail or "").lower()
