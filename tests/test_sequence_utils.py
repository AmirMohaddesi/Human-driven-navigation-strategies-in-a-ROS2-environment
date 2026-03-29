"""Unit tests for agent/sequence_utils.py (no ROS)."""

from __future__ import annotations

from typing import Any, Dict, List
from unittest.mock import patch

from multi_robot_mission_stack.agent.sequence_utils import run_sequential_named_navigation


def test_invalid_steps_returns_failure_summary() -> None:
    class _F:
        def handle_command(self, _c: Dict[str, Any]) -> Dict[str, Any]:
            raise AssertionError("should not be called")

    out = run_sequential_named_navigation(
        _F(),  # type: ignore[arg-type]
        "not-a-list",  # type: ignore[arg-type]
        per_goal_timeout_sec=5.0,
        poll_interval_sec=0.1,
    )
    assert out["overall_outcome"] == "failure"
    assert out["steps_run"] == 0
    assert "error" in out


def test_two_steps_both_succeed() -> None:
    nav_seq = [
        {"status": "accepted", "goal_id": "g1", "nav_status": "submitted", "message": ""},
        {"status": "accepted", "goal_id": "g2", "nav_status": "submitted", "message": ""},
    ]

    class _F:
        def __init__(self) -> None:
            self._i = 0

        def handle_command(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
            if cmd.get("type") == "navigate":
                r = nav_seq[self._i]
                self._i += 1
                return dict(r)
            return {"status": "success", "nav_status": "in_progress", "message": ""}

    fac = _F()

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
        "multi_robot_mission_stack.agent.sequence_utils.wait_for_terminal_navigation_state",
        side_effect=_fake_wait,
    ):
        out = run_sequential_named_navigation(
            fac,  # type: ignore[arg-type]
            [
                {"robot_id": "robot1", "location_name": "base"},
                {"robot_id": "robot1", "location_name": "test_goal"},
            ],
            per_goal_timeout_sec=5.0,
            poll_interval_sec=0.1,
        )

    assert out["overall_outcome"] == "success"
    assert out["total_steps"] == 2
    assert out["steps_run"] == 2
    assert out["succeeded_count"] == 2
    assert out["failed_count"] == 0
    assert out["stopped_early"] is False
    assert all(s["step_outcome"] == "succeeded" for s in out["steps"])


def test_stop_on_first_failure() -> None:
    class _F:
        def handle_command(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
            if cmd.get("type") == "navigate":
                return {"status": "failed", "nav_status": "unknown", "message": "denied"}
            return {}

    out = run_sequential_named_navigation(
        _F(),  # type: ignore[arg-type]
        [
            {"robot_id": "robot1", "location_name": "base"},
            {"robot_id": "robot1", "location_name": "test_goal"},
        ],
        per_goal_timeout_sec=5.0,
        poll_interval_sec=0.1,
        continue_on_failure=False,
    )
    assert out["overall_outcome"] == "failure"
    assert out["steps_run"] == 1
    assert out["stopped_early"] is True
    assert out["failed_count"] == 1


def test_continue_on_failure_runs_second() -> None:
    nav_seq: List[Dict[str, Any]] = [
        {"status": "failed", "goal_id": None, "nav_status": "unknown", "message": "x"},
        {"status": "accepted", "goal_id": "g2", "nav_status": "submitted", "message": ""},
    ]

    class _F:
        def __init__(self) -> None:
            self._i = 0

        def handle_command(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
            if cmd.get("type") == "navigate":
                r = nav_seq[self._i]
                self._i += 1
                return dict(r)
            return {}

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
        "multi_robot_mission_stack.agent.sequence_utils.wait_for_terminal_navigation_state",
        side_effect=_fake_wait,
    ):
        out = run_sequential_named_navigation(
            _F(),  # type: ignore[arg-type]
            [
                {"robot_id": "robot1", "location_name": "a"},
                {"robot_id": "robot1", "location_name": "b"},
            ],
            per_goal_timeout_sec=5.0,
            poll_interval_sec=0.1,
            continue_on_failure=True,
        )

    assert out["steps_run"] == 2
    assert out["succeeded_count"] == 1
    assert out["failed_count"] == 1
    assert out["stopped_early"] is False
