"""V5.1 — pure classification helper (no facade)."""

from __future__ import annotations

from multi_robot_mission_stack.agent.navigate_failure_classification_v51 import (
    NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE,
    navigate_failure_kind,
)
from multi_robot_mission_stack.semantic.blocked_passage_v301 import (
    BLOCKED_OUTCOME_SCHEMA_VERSION,
    BLOCKED_OUTCOME_VALUE,
    BLOCKED_PEER_BELIEF_FAILURE_MESSAGE,
    BLOCKED_REASON_CODE,
)


def test_navigate_failure_kind_advisory_blocked_match() -> None:
    nav = {
        "outcome": BLOCKED_OUTCOME_VALUE,
        "schema_version": BLOCKED_OUTCOME_SCHEMA_VERSION,
        "reason_code": BLOCKED_REASON_CODE,
        "requested_location_name": "base",
        "active_belief_ids": ["a"],
        "status": "failed",
        "message": "navigation target blocked by peer belief",
        "nav_status": "unknown",
        "goal_id": None,
    }
    assert navigate_failure_kind(nav) == NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE


def test_navigate_failure_kind_wrong_reason_code() -> None:
    nav = {
        "outcome": BLOCKED_OUTCOME_VALUE,
        "reason_code": "other",
        "status": "failed",
        "goal_id": None,
    }
    assert navigate_failure_kind(nav) is None


def test_navigate_failure_kind_generic_failed() -> None:
    assert navigate_failure_kind({"status": "failed", "message": "denied"}) is None


def test_v52_navigate_failure_kind_bridge_normalized_shape() -> None:
    """MissionClient + bridge surface: no outcome/reason_code on the wire."""
    nav = {
        "status": "failed",
        "message": BLOCKED_PEER_BELIEF_FAILURE_MESSAGE,
        "nav_status": "unknown",
        "goal_id": "",
    }
    assert navigate_failure_kind(nav) == NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE


def test_v52_navigate_failure_kind_bridge_shape_goal_id_none() -> None:
    nav = {
        "status": "failed",
        "message": BLOCKED_PEER_BELIEF_FAILURE_MESSAGE,
        "nav_status": "unknown",
        "goal_id": None,
    }
    assert navigate_failure_kind(nav) == NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE


def test_v52_navigate_failure_kind_does_not_infer_if_reason_present_but_wrong() -> None:
    nav = {
        "outcome": BLOCKED_OUTCOME_VALUE,
        "reason_code": "other",
        "status": "failed",
        "message": BLOCKED_PEER_BELIEF_FAILURE_MESSAGE,
        "goal_id": "",
    }
    assert navigate_failure_kind(nav) is None
