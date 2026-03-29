"""Unit tests for scripts/wait_for_goal.py terminal detection."""

from __future__ import annotations

import importlib.util
from pathlib import Path

_MOD_PATH = Path(__file__).resolve().parents[1] / "scripts" / "wait_for_goal.py"
_spec = importlib.util.spec_from_file_location("wait_for_goal", _MOD_PATH)
assert _spec and _spec.loader
_wfg = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_wfg)


def test_terminal_outcome_nav_statuses() -> None:
    assert _wfg.terminal_outcome({"nav_status": "succeeded", "status": "success"}) == "succeeded"
    assert _wfg.terminal_outcome({"nav_status": "cancelled", "status": "success"}) == "cancelled"
    assert _wfg.terminal_outcome({"nav_status": "failed", "status": "success"}) == "failed"
    assert _wfg.terminal_outcome({"nav_status": "rejected", "status": "failure"}) == "rejected"
    assert _wfg.terminal_outcome({"nav_status": "not_found", "status": "failure"}) == "not_found"


def test_terminal_outcome_failure_not_found_message() -> None:
    assert (
        _wfg.terminal_outcome(
            {
                "status": "failure",
                "nav_status": "unknown",
                "message": "Goal id not found for this robot",
            }
        )
        == "not_found"
    )


def test_terminal_outcome_in_progress_not_terminal() -> None:
    assert (
        _wfg.terminal_outcome(
            {"status": "success", "nav_status": "in_progress", "message": "..."}
        )
        is None
    )
