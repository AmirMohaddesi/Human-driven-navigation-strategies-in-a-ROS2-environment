"""End-to-end tests for ``MissionAgentFacade`` with mock stack."""

from __future__ import annotations

import pytest

from multi_robot_mission_stack.agent.mission_agent_facade import MissionAgentFacade
from multi_robot_mission_stack.agent.mission_policy import MissionPolicy
from multi_robot_mission_stack.agent.policy_config import MissionPolicyConfig


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
