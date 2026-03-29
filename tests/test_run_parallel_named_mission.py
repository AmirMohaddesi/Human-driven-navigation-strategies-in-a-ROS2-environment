"""Unit tests for scripts/run_parallel_named_mission.py (no subprocess)."""

from __future__ import annotations

import importlib.util
from pathlib import Path

_MOD = Path(__file__).resolve().parents[1] / "scripts" / "run_parallel_named_mission.py"
_spec = importlib.util.spec_from_file_location("run_parallel_named_mission", _MOD)
assert _spec and _spec.loader
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)


def test_duplicate_robot_error() -> None:
    assert _mod.duplicate_robot_error([("robot1", "a"), ("robot2", "b")]) is None
    err = _mod.duplicate_robot_error([("robot1", "a"), ("robot1", "b")])
    assert err is not None
    assert "robot1" in err


def test_finalize_step_outcome_succeeded() -> None:
    rec = {
        "nav_exit": 0,
        "goal_id": "g1",
        "wait_exit": 0,
        "wait_detail": {"outcome": "succeeded"},
        "error": None,
    }
    _mod.finalize_step_outcome(rec)
    assert rec["step_outcome"] == "succeeded"
    assert rec["error"] is None


def test_finalize_step_outcome_wait_not_succeeded() -> None:
    rec = {
        "nav_exit": 0,
        "goal_id": "g1",
        "wait_exit": 0,
        "wait_detail": {"outcome": "cancelled"},
        "error": None,
    }
    _mod.finalize_step_outcome(rec)
    assert rec["step_outcome"] == "failed"


def test_finalize_step_outcome_no_goal() -> None:
    rec = {"nav_exit": 0, "goal_id": None, "error": "navigate_missing_goal_id"}
    _mod.finalize_step_outcome(rec)
    assert rec["step_outcome"] == "failed"
