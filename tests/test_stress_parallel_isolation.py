"""Pure isolation checks for scripts/stress_parallel_named_mission.py (no ROS)."""

from __future__ import annotations

import importlib.util
from pathlib import Path

_MOD = Path(__file__).resolve().parents[1] / "scripts" / "stress_parallel_named_mission.py"
_spec = importlib.util.spec_from_file_location("stress_parallel_named_mission", _MOD)
assert _spec and _spec.loader
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)


def test_isolation_check_ok() -> None:
    summary = {
        "steps": [
            {
                "index": 0,
                "robot_id": "robot1",
                "location_name": "base",
                "goal_id": "a",
            },
            {
                "index": 1,
                "robot_id": "robot2",
                "location_name": "test_goal",
                "goal_id": "b",
            },
        ]
    }
    assert _mod.isolation_check(summary, [("robot1", "base"), ("robot2", "test_goal")]) == []


def test_isolation_check_duplicate_goal_id() -> None:
    summary = {
        "steps": [
            {"index": 0, "robot_id": "robot1", "location_name": "base", "goal_id": "same"},
            {"index": 1, "robot_id": "robot2", "location_name": "x", "goal_id": "same"},
        ]
    }
    issues = _mod.isolation_check(summary, [("robot1", "base"), ("robot2", "x")])
    assert any("duplicate_goal_id" in i for i in issues)


def test_count_timeouts_and_cancel() -> None:
    summary = {
        "steps": [
            {"wait_detail": {"outcome": "timeout"}},
            {"wait_detail": {"outcome": "cancelled"}},
            {"error": "subprocess_timeout"},
        ]
    }
    t, c = _mod.count_timeouts_and_cancel(summary)
    assert t == 2
    assert c == 1
