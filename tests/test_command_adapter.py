"""Tests for ``CommandAdapter`` (external command → internal graph request)."""

from __future__ import annotations

import pytest

from multi_robot_mission_stack.agent.command_adapter import CommandAdapter


@pytest.fixture
def adapter() -> CommandAdapter:
    return CommandAdapter()


def test_valid_navigate_pose_command_adapts_correctly(adapter: CommandAdapter) -> None:
    cmd = {
        "type": "navigate",
        "target": "pose",
        "robot_id": "robot1",
        "x": 1.0,
        "y": 2.0,
        "yaw": 0.0,
    }
    out = adapter.adapt(cmd)
    assert out == {
        "action": "navigate_to_pose",
        "robot_id": "robot1",
        "x": 1.0,
        "y": 2.0,
        "yaw": 0.0,
    }


def test_valid_navigate_command_adapts_correctly(adapter: CommandAdapter) -> None:
    cmd = {
        "type": "navigate",
        "target": "named_location",
        "robot_id": "robot1",
        "location_name": "base",
    }
    out = adapter.adapt(cmd)
    assert out == {
        "action": "navigate_to_named_location",
        "robot_id": "robot1",
        "location_name": "base",
    }


def test_valid_query_command_adapts_correctly(adapter: CommandAdapter) -> None:
    cmd = {
        "type": "query",
        "target": "navigation_state",
        "robot_id": "robot1",
        "goal_id": "mock-goal-001",
    }
    out = adapter.adapt(cmd)
    assert out == {
        "action": "get_navigation_state",
        "robot_id": "robot1",
        "goal_id": "mock-goal-001",
    }


def test_valid_cancel_command_adapts_correctly(adapter: CommandAdapter) -> None:
    cmd = {
        "type": "cancel",
        "target": "navigation",
        "robot_id": "robot1",
        "goal_id": "mock-goal-001",
    }
    out = adapter.adapt(cmd)
    assert out == {
        "action": "cancel_navigation",
        "robot_id": "robot1",
        "goal_id": "mock-goal-001",
    }


def test_non_dict_command_returns_failure_dict(adapter: CommandAdapter) -> None:
    out = adapter.adapt("not-a-dict")  # type: ignore[arg-type]
    assert out["status"] == "failed"
    assert out["nav_status"] == "unknown"
    assert "dict" in out["message"].lower()


def test_unsupported_navigate_target_returns_failure_dict(adapter: CommandAdapter) -> None:
    cmd = {
        "type": "navigate",
        "target": "unknown_target",
        "robot_id": "robot1",
        "location_name": "base",
    }
    out = adapter.adapt(cmd)
    assert out["status"] == "failed"
    assert out["nav_status"] == "unknown"
    assert "named_location" in out["message"] or "pose" in out["message"]


def test_missing_robot_id_returns_failure_dict(adapter: CommandAdapter) -> None:
    cmd = {
        "type": "navigate",
        "target": "named_location",
        "location_name": "base",
    }
    out = adapter.adapt(cmd)
    assert out["status"] == "failed"
    assert out["nav_status"] == "unknown"
    assert "robot_id" in out["message"].lower()


def test_missing_location_name_returns_failure_dict(adapter: CommandAdapter) -> None:
    cmd = {
        "type": "navigate",
        "target": "named_location",
        "robot_id": "robot1",
    }
    out = adapter.adapt(cmd)
    assert out["status"] == "failed"
    assert out["nav_status"] == "unknown"
    assert "location_name" in out["message"].lower()


def test_missing_goal_id_returns_failure_dict(adapter: CommandAdapter) -> None:
    cmd = {
        "type": "query",
        "target": "navigation_state",
        "robot_id": "robot1",
    }
    out = adapter.adapt(cmd)
    assert out["status"] == "failed"
    assert out["nav_status"] == "unknown"
    assert "goal_id" in out["message"].lower()


def test_adapt_and_validate_matches_adapt_for_success(adapter: CommandAdapter) -> None:
    cmd = {
        "type": "query",
        "target": "navigation_state",
        "robot_id": "robot2",
        "goal_id": "g-1",
    }
    assert adapter.adapt_and_validate(cmd) == adapter.adapt(cmd)
