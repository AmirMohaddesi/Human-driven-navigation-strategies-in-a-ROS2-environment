"""Tests for ``MissionGraph`` with mock client and default policy (internal requests)."""

from __future__ import annotations

import pytest

from multi_robot_mission_stack.agent.mission_graph import MissionGraph
from multi_robot_mission_stack.agent.mission_policy import MissionPolicy
from multi_robot_mission_stack.agent.mission_tools import MissionTools
from multi_robot_mission_stack.agent.mock_mission_client import MockMissionClient
from multi_robot_mission_stack.agent.policy_config import build_default_policy_config


@pytest.fixture
def graph() -> MissionGraph:
    client = MockMissionClient()
    tools = MissionTools(client)
    policy = MissionPolicy(build_default_policy_config())
    return MissionGraph(tools, policy=policy)


def test_navigate_to_pose_succeeds(graph: MissionGraph) -> None:
    req = {
        "action": "navigate_to_pose",
        "robot_id": "robot1",
        "x": 0.5,
        "y": -0.25,
        "yaw": 1.57,
    }
    out = graph.invoke(req)
    assert out["status"] == "accepted"
    assert out["nav_status"] == "submitted"
    assert out.get("goal_id")


def test_navigate_to_named_location_succeeds(graph: MissionGraph) -> None:
    req = {
        "action": "navigate_to_named_location",
        "robot_id": "robot1",
        "location_name": "base",
    }
    out = graph.invoke(req)
    assert out["status"] == "accepted"
    assert out["nav_status"] == "submitted"
    assert out.get("goal_id")


def test_cancel_navigation_succeeds_after_submitted_goal(graph: MissionGraph) -> None:
    nav = graph.invoke(
        {
            "action": "navigate_to_named_location",
            "robot_id": "robot1",
            "location_name": "base",
        }
    )
    gid = nav["goal_id"]
    out = graph.invoke(
        {
            "action": "cancel_navigation",
            "robot_id": "robot1",
            "goal_id": gid,
        }
    )
    assert out["status"] == "success"
    assert out["nav_status"] == "canceled"


def test_get_navigation_state_succeeds_after_submitted_goal(graph: MissionGraph) -> None:
    nav = graph.invoke(
        {
            "action": "navigate_to_named_location",
            "robot_id": "robot1",
            "location_name": "test_goal",
        }
    )
    gid = nav["goal_id"]
    state = graph.invoke(
        {
            "action": "get_navigation_state",
            "robot_id": "robot1",
            "goal_id": gid,
        }
    )
    assert state["status"] == "success"
    assert state["nav_status"] == "in_progress"


def test_unsupported_internal_action_returns_failed_dict(graph: MissionGraph) -> None:
    """
    Internal actions outside the policy allowlist are rejected (policy denial).
    """
    req = {
        "action": "idle_forever",
        "robot_id": "robot1",
        "location_name": "base",
    }
    out = graph.invoke(req)
    assert out["status"] == "failed"
    assert out["nav_status"] == "unknown"
    assert "policy denied" in out["message"].lower()


def test_missing_action_in_request_returns_failed_dict(graph: MissionGraph) -> None:
    req = {
        "robot_id": "robot1",
        "location_name": "base",
    }
    out = graph.invoke(req)
    assert out["status"] == "failed"
    assert "action" in out["message"].lower()


def test_policy_denial_returns_failed_dict_with_unknown_nav_status(
    graph: MissionGraph,
) -> None:
    req = {
        "action": "navigate_to_named_location",
        "robot_id": "robot1",
        "location_name": "not_a_real_site",
    }
    out = graph.invoke(req)
    assert out["status"] == "failed"
    assert out["nav_status"] == "unknown"
    assert "policy denied" in out["message"].lower()
