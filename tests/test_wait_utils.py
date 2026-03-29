"""Unit tests for agent/wait_utils.py (no ROS)."""

from __future__ import annotations

from typing import Any, Dict, List

from multi_robot_mission_stack.agent.wait_utils import (
    terminal_outcome,
    wait_for_terminal_navigation_state,
)


def test_terminal_outcome_from_nav_status() -> None:
    assert terminal_outcome({"nav_status": "succeeded", "status": "success"}) == "succeeded"
    assert terminal_outcome({"nav_status": "cancelled", "status": "success"}) == "cancelled"
    assert terminal_outcome({"nav_status": "in_progress", "status": "success"}) is None


def test_terminal_outcome_failure_and_not_found() -> None:
    assert (
        terminal_outcome(
            {"status": "failure", "nav_status": "unknown", "message": "Goal id not found"}
        )
        == "not_found"
    )
    assert terminal_outcome({"status": "failure", "nav_status": "unknown", "message": "other"}) == "failed"


class _SeqFacade:
    def __init__(self, responses: List[Dict[str, Any]]) -> None:
        self._responses = list(responses)

    def handle_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        assert command.get("type") == "query"
        if not self._responses:
            return {"status": "success", "nav_status": "succeeded", "message": ""}
        return self._responses.pop(0)


def test_wait_terminates_on_succeeded() -> None:
    fac = _SeqFacade(
        [
            {"status": "success", "nav_status": "in_progress", "message": ""},
            {"status": "success", "nav_status": "succeeded", "message": ""},
        ]
    )
    out = wait_for_terminal_navigation_state(
        fac, "robot1", "g1", timeout_sec=5.0, poll_interval_sec=0.1
    )
    assert out["outcome"] == "succeeded"
    assert out["polls"] == 2
    assert out["robot_id"] == "robot1"
    assert out["goal_id"] == "g1"


def test_wait_adapter_failure_immediate() -> None:
    fac = _SeqFacade([{"status": "failed", "nav_status": "unknown", "message": "policy denied"}])
    out = wait_for_terminal_navigation_state(
        fac, "r", "g", timeout_sec=5.0, poll_interval_sec=0.1
    )
    assert out["outcome"] == "failed"
    assert out["polls"] == 1


def test_wait_handle_command_raises() -> None:
    class _Bad:
        def handle_command(self, _cmd: Dict[str, Any]) -> Dict[str, Any]:
            raise RuntimeError("boom")

    out = wait_for_terminal_navigation_state(
        _Bad(), "r", "g", timeout_sec=5.0, poll_interval_sec=0.1
    )
    assert out["outcome"] == "failed"
    assert "boom" in out["message"]


def test_wait_timeout_zero_no_poll() -> None:
    fac = _SeqFacade([])
    out = wait_for_terminal_navigation_state(
        fac, "r", "g", timeout_sec=0.0, poll_interval_sec=0.1
    )
    assert out["outcome"] == "timeout"
    assert out["polls"] == 0
