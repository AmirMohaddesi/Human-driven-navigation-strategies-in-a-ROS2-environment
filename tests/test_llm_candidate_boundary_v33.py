"""Unit tests for V3.3 llm_context + llm_candidate boundary (no ROS)."""

from __future__ import annotations

import json

import pytest

from multi_robot_mission_stack.semantic.llm_candidate_boundary_v33 import (
    CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH,
    CANDIDATE_MALFORMED,
    CANDIDATE_MISSING_OR_EXTRA_KEYS,
    CANDIDATE_SCHEMA_MISMATCH,
    CANDIDATE_SEMANTICALLY_INSUFFICIENT,
    CONTEXT_INVALID,
    evaluate_llm_candidate_boundary,
)


def _ctx(**overrides):
    base = {
        "schema_version": "v3.3.llm_context.1",
        "location_ref": "base",
        "source_robot_id": "r1",
        "nav_goal_status": "active",
        "stall_duration_sec": 12.0,
        "planner_status": "computing",
        "lidar_occlusion_proxy": True,
        "operator_hint": "",
    }
    base.update(overrides)
    return base


def test_accept_minimal_candidate():
    ctx = _ctx()
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
    assert r.mapped.location_ref == "base"
    assert r.mapped.source_robot_id == "r1"
    assert r.mapped.sensor_class == "lidar_occlusion"
    assert r.mapped.rationale_short is None


def test_accept_with_rationale():
    ctx = _ctx()
    out = {
        "schema_version": "v3.3.llm_candidate.1",
        "assert_blocked": True,
        "location_ref": "base",
        "confidence": 1.0,
        "ttl_sec": 60,
        "sensor_class": "x",
        "rationale_short": "ok",
    }
    r = evaluate_llm_candidate_boundary(ctx, json.dumps(out))
    assert r.accepted is True
    assert r.mapped.rationale_short == "ok"


def test_context_extra_key_rejects():
    c = _ctx()
    c["extra"] = 1
    r = evaluate_llm_candidate_boundary(c, "{}")
    assert r.accepted is False
    assert r.reason == CONTEXT_INVALID


def test_context_bad_nav_goal():
    r = evaluate_llm_candidate_boundary(_ctx(nav_goal_status="bogus"), "{}")
    assert r.reason == CONTEXT_INVALID


def test_context_operator_hint_length():
    r = evaluate_llm_candidate_boundary(
        _ctx(operator_hint="x" * 201),
        "{}",
    )
    assert r.reason == CONTEXT_INVALID


def test_context_operator_hint_required_string():
    r = evaluate_llm_candidate_boundary(_ctx(operator_hint=None), "{}")  # type: ignore[arg-type]
    assert r.reason == CONTEXT_INVALID


def test_candidate_markdown_rejects():
    r = evaluate_llm_candidate_boundary(
        _ctx(),
        '```json\n{"schema_version":"v3.3.llm_candidate.1","assert_blocked":false}\n```',
    )
    assert r.reason == CANDIDATE_MALFORMED


def test_candidate_trailing_garbage():
    r = evaluate_llm_candidate_boundary(
        _ctx(),
        '{"schema_version":"v3.3.llm_candidate.1","assert_blocked":false} x',
    )
    assert r.reason == CANDIDATE_MALFORMED


def test_assert_blocked_false_only_two_keys():
    s = json.dumps(
        {
            "schema_version": "v3.3.llm_candidate.1",
            "assert_blocked": False,
        }
    )
    r = evaluate_llm_candidate_boundary(_ctx(), s)
    assert r.accepted is False
    assert r.reason == CANDIDATE_SEMANTICALLY_INSUFFICIENT


def test_assert_blocked_false_extra_key():
    s = json.dumps(
        {
            "schema_version": "v3.3.llm_candidate.1",
            "assert_blocked": False,
            "location_ref": "base",
        }
    )
    r = evaluate_llm_candidate_boundary(_ctx(), s)
    assert r.reason == CANDIDATE_MISSING_OR_EXTRA_KEYS


def test_schema_mismatch():
    s = json.dumps(
        {
            "schema_version": "v3.3.llm_candidate.2",
            "assert_blocked": True,
            "location_ref": "base",
            "confidence": 0.5,
            "ttl_sec": 1.0,
            "sensor_class": "a",
        }
    )
    r = evaluate_llm_candidate_boundary(_ctx(), s)
    assert r.reason == CANDIDATE_SCHEMA_MISMATCH


def test_location_mismatch():
    s = json.dumps(
        {
            "schema_version": "v3.3.llm_candidate.1",
            "assert_blocked": True,
            "location_ref": "other",
            "confidence": 0.5,
            "ttl_sec": 1.0,
            "sensor_class": "a",
        }
    )
    r = evaluate_llm_candidate_boundary(_ctx(), s)
    assert r.reason == CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH


def test_confidence_bool_rejected():
    s = json.dumps(
        {
            "schema_version": "v3.3.llm_candidate.1",
            "assert_blocked": True,
            "location_ref": "base",
            "confidence": True,
            "ttl_sec": 1.0,
            "sensor_class": "a",
        }
    )
    r = evaluate_llm_candidate_boundary(_ctx(), s)
    assert r.reason == CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH


def test_ttl_zero_rejected():
    s = json.dumps(
        {
            "schema_version": "v3.3.llm_candidate.1",
            "assert_blocked": True,
            "location_ref": "base",
            "confidence": 0.5,
            "ttl_sec": 0.0,
            "sensor_class": "a",
        }
    )
    r = evaluate_llm_candidate_boundary(_ctx(), s)
    assert r.reason == CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH


def test_sensor_class_uppercase_rejected():
    s = json.dumps(
        {
            "schema_version": "v3.3.llm_candidate.1",
            "assert_blocked": True,
            "location_ref": "base",
            "confidence": 0.5,
            "ttl_sec": 1.0,
            "sensor_class": "Lidar",
        }
    )
    r = evaluate_llm_candidate_boundary(_ctx(), s)
    assert r.reason == CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH


@pytest.mark.parametrize(
    "whitespace",
    ["  ", "\n", "\t"],
)
def test_whitespace_only_output(whitespace):
    r = evaluate_llm_candidate_boundary(_ctx(), whitespace)
    assert r.reason == CANDIDATE_MALFORMED


def test_strips_once_valid():
    inner = {
        "schema_version": "v3.3.llm_candidate.1",
        "assert_blocked": True,
        "location_ref": "base",
        "confidence": 0.5,
        "ttl_sec": 10.0,
        "sensor_class": "a",
    }
    r = evaluate_llm_candidate_boundary(_ctx(), "  \n" + json.dumps(inner) + "\n  ")
    assert r.accepted is True
