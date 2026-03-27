"""Tests for ``MissionPolicy`` and default ``MissionPolicyConfig``."""

from __future__ import annotations

import pytest

from multi_robot_mission_stack.agent.mission_policy import MissionPolicy
from multi_robot_mission_stack.agent.policy_config import (
    MissionPolicyConfig,
    build_default_policy_config,
)


@pytest.fixture
def default_policy() -> MissionPolicy:
    return MissionPolicy(build_default_policy_config())


def test_default_policy_allows_valid_navigate_request(default_policy: MissionPolicy) -> None:
    req = {
        "action": "navigate_to_named_location",
        "robot_id": "robot1",
        "location_name": "base",
    }
    verdict = default_policy.evaluate(req)
    assert verdict["allowed"] is True
    assert verdict["message"] == "request allowed"


def test_default_policy_allows_valid_state_query(default_policy: MissionPolicy) -> None:
    req = {
        "action": "get_navigation_state",
        "robot_id": "robot2",
        "goal_id": "mock-goal-001",
    }
    verdict = default_policy.evaluate(req)
    assert verdict["allowed"] is True
    assert verdict["message"] == "request allowed"


def test_denies_unknown_action(default_policy: MissionPolicy) -> None:
    req = {
        "action": "cancel_everything",
        "robot_id": "robot1",
        "location_name": "base",
    }
    verdict = default_policy.evaluate(req)
    assert verdict["allowed"] is False
    assert "cancel_everything" in verdict["message"] or "not permitted" in verdict["message"]


def test_denies_unknown_robot_id(default_policy: MissionPolicy) -> None:
    req = {
        "action": "navigate_to_named_location",
        "robot_id": "robot99",
        "location_name": "base",
    }
    verdict = default_policy.evaluate(req)
    assert verdict["allowed"] is False
    assert "robot99" in verdict["message"]
    assert "not permitted" in verdict["message"]


def test_denies_unknown_location_name(default_policy: MissionPolicy) -> None:
    req = {
        "action": "navigate_to_named_location",
        "robot_id": "robot1",
        "location_name": "forbidden_hangar",
    }
    verdict = default_policy.evaluate(req)
    assert verdict["allowed"] is False
    assert "forbidden_hangar" in verdict["message"]
    assert "not permitted" in verdict["message"]


def test_denies_missing_goal_id_for_get_navigation_state(
    default_policy: MissionPolicy,
) -> None:
    req = {
        "action": "get_navigation_state",
        "robot_id": "robot1",
    }
    verdict = default_policy.evaluate(req)
    assert verdict["allowed"] is False
    assert "goal_id" in verdict["message"].lower()
