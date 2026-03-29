"""Unit tests for agent/parallel_utils.py (no ROS)."""

from __future__ import annotations

from typing import Any, Dict, List
from unittest.mock import patch

from multi_robot_mission_stack.agent.parallel_utils import (
    duplicate_robot_error,
    run_parallel_named_navigation,
)


def test_invalid_steps() -> None:
    out = run_parallel_named_navigation(
        object(),  # type: ignore[arg-type]
        "bad",  # type: ignore[arg-type]
        per_goal_timeout_sec=5.0,
        poll_interval_sec=0.1,
    )
    assert out["overall_outcome"] == "failure"
    assert out["steps"] == []
    assert "error" in out


def test_duplicate_robot_rejected() -> None:
    out = run_parallel_named_navigation(
        object(),  # type: ignore[arg-type]
        [
            {"robot_id": "robot1", "location_name": "a"},
            {"robot_id": "robot1", "location_name": "b"},
        ],
        per_goal_timeout_sec=5.0,
        poll_interval_sec=0.1,
    )
    assert out["overall_outcome"] == "failure"
    assert "duplicate" in out["error"]


def test_duplicate_robot_error_helper() -> None:
    assert duplicate_robot_error(
        [
            {"robot_id": "r1", "location_name": "x"},
            {"robot_id": "r2", "location_name": "y"},
        ]
    ) is None
    err = duplicate_robot_error(
        [
            {"robot_id": "r1", "location_name": "a"},
            {"robot_id": "r1", "location_name": "b"},
        ]
    )
    assert err is not None


def test_parallel_both_succeed() -> None:
    nav_by_index = {
        0: {"status": "accepted", "goal_id": "g0", "nav_status": "submitted", "message": ""},
        1: {"status": "accepted", "goal_id": "g1", "nav_status": "submitted", "message": ""},
    }

    class _F:
        def handle_command(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
            loc = cmd.get("location_name")
            if loc == "base":
                return dict(nav_by_index[0])
            if loc == "test_goal":
                return dict(nav_by_index[1])
            return {"status": "failed", "message": "bad"}

    def _fake_wait(
        _fac: Any,
        robot_id: str,
        goal_id: str,
        *,
        timeout_sec: float,
        poll_interval_sec: float,
    ) -> Dict[str, Any]:
        return {
            "outcome": "succeeded",
            "robot_id": robot_id,
            "goal_id": goal_id,
            "status": "success",
            "nav_status": "succeeded",
            "message": "",
            "polls": 1,
            "elapsed_sec": 0.01,
        }

    steps: List[Dict[str, str]] = [
        {"robot_id": "robot1", "location_name": "base"},
        {"robot_id": "robot2", "location_name": "test_goal"},
    ]

    with patch(
        "multi_robot_mission_stack.agent.parallel_utils.wait_for_terminal_navigation_state",
        side_effect=_fake_wait,
    ):
        out = run_parallel_named_navigation(
            _F(),  # type: ignore[arg-type]
            steps,
            per_goal_timeout_sec=5.0,
            poll_interval_sec=0.1,
        )

    assert out["overall_outcome"] == "success"
    assert out["total_steps"] == 2
    assert out["submitted_count"] == 2
    assert out["succeeded_count"] == 2
    assert out["failed_count"] == 0


def test_parallel_one_nav_fails() -> None:
    class _F:
        def handle_command(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
            if cmd.get("location_name") == "bad":
                return {"status": "failed", "nav_status": "unknown", "message": "x"}
            return {"status": "accepted", "goal_id": "gok", "nav_status": "submitted", "message": ""}

    def _fake_wait(
        _fac: Any,
        robot_id: str,
        goal_id: str,
        *,
        timeout_sec: float,
        poll_interval_sec: float,
    ) -> Dict[str, Any]:
        return {
            "outcome": "succeeded",
            "robot_id": robot_id,
            "goal_id": goal_id,
            "status": "success",
            "nav_status": "succeeded",
            "message": "",
            "polls": 1,
            "elapsed_sec": 0.01,
        }

    with patch(
        "multi_robot_mission_stack.agent.parallel_utils.wait_for_terminal_navigation_state",
        side_effect=_fake_wait,
    ):
        out = run_parallel_named_navigation(
            _F(),  # type: ignore[arg-type]
            [
                {"robot_id": "robot1", "location_name": "base"},
                {"robot_id": "robot2", "location_name": "bad"},
            ],
            per_goal_timeout_sec=5.0,
            poll_interval_sec=0.1,
        )

    assert out["overall_outcome"] == "failure"
    assert out["submitted_count"] == 1
    assert out["succeeded_count"] == 1
    assert out["failed_count"] == 1
