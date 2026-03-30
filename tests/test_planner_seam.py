"""Tests for planner seam (intent → v1 spec → inspect_checked → execute_checked)."""

from __future__ import annotations

from typing import Any, Dict

from multi_robot_mission_stack.planner_seam import (
    build_team_named_mission_spec_v1_from_intent,
    run_planner_team_named_mission_checked,
)


def test_build_team_named_mission_spec_v1_from_intent_valid_sequence() -> None:
    intent = {
        "mode": "sequence",
        "steps": [{"robot_id": "r1", "location_name": "base"}],
    }
    b = build_team_named_mission_spec_v1_from_intent(intent)
    assert b["ok"] is True
    assert b["spec"]["version"] == "v1"
    assert b["spec"]["mode"] == "sequence"
    assert b["spec"]["steps"] == intent["steps"]
    assert "options" not in b["spec"]


def test_build_team_named_mission_spec_v1_from_intent_with_options() -> None:
    intent = {
        "mode": "parallel",
        "steps": [
            {"robot_id": "a", "location_name": "x"},
            {"robot_id": "b", "location_name": "y"},
        ],
        "options": {"max_workers": 2},
    }
    b = build_team_named_mission_spec_v1_from_intent(intent)
    assert b["ok"] is True
    assert b["spec"]["options"] == {"max_workers": 2}


def test_build_team_named_mission_spec_v1_from_intent_not_dict() -> None:
    b = build_team_named_mission_spec_v1_from_intent([])  # type: ignore[arg-type]
    assert b["ok"] is False
    assert b["spec"] == {}
    assert "intent must be a dict" in b["errors"]


def test_build_team_named_mission_spec_v1_from_intent_contract_parallel_dup() -> None:
    intent = {
        "mode": "parallel",
        "steps": [
            {"robot_id": "r1", "location_name": "a"},
            {"robot_id": "r1", "location_name": "b"},
        ],
    }
    b = build_team_named_mission_spec_v1_from_intent(intent)
    assert b["ok"] is False
    assert any("duplicate" in e.lower() for e in b["errors"])


def test_run_planner_team_named_mission_checked_happy_path() -> None:
    intent = {"steps": [{"robot_id": "r1", "location_name": "base"}]}

    def _insp(_spec: Any) -> Dict[str, Any]:
        return {"ok": True, "version": "v1", "overall_outcome": "success", "summary": {"message": "i", "error": None}}

    def _exec(_spec: Any) -> Dict[str, Any]:
        return {"ok": True, "version": "v1", "overall_outcome": "success", "summary": {"message": "e", "error": None}}

    out = run_planner_team_named_mission_checked(intent, inspect_checked=_insp, execute_checked=_exec)
    assert out["version"] == "v1"
    assert out["ok"] is True
    assert out["overall_outcome"] == "success"
    assert out["inspection"] is not None
    assert out["execution"] is not None


def test_run_planner_team_named_mission_checked_stops_on_contract_failure() -> None:
    def _fail(_spec: Any) -> Dict[str, Any]:
        raise AssertionError("should not call inspect")

    out = run_planner_team_named_mission_checked(
        {"mode": "parallel", "steps": [{"robot_id": "x", "location_name": "a"}, {"robot_id": "x", "location_name": "b"}]},
        inspect_checked=_fail,
        execute_checked=_fail,
    )
    assert out["ok"] is False
    assert out["inspection"] is None
    assert out["execution"] is None


def test_run_planner_team_named_mission_checked_stops_on_inspect_failure() -> None:
    intent = {"steps": [{"robot_id": "r1", "location_name": "base"}]}

    def _insp(_spec: Any) -> Dict[str, Any]:
        return {"ok": False, "version": "v1", "overall_outcome": "error", "summary": {"message": "", "error": "bad"}}

    def _exec(_spec: Any) -> Dict[str, Any]:
        raise AssertionError("should not call execute")

    out = run_planner_team_named_mission_checked(intent, inspect_checked=_insp, execute_checked=_exec)
    assert out["ok"] is False
    assert out["inspection"] is not None
    assert out["execution"] is None
    assert out["summary"]["error"] == "bad"


def test_run_planner_team_named_mission_checked_execute_failure_surfaces() -> None:
    intent = {"steps": [{"robot_id": "r1", "location_name": "base"}]}

    def _insp(_spec: Any) -> Dict[str, Any]:
        return {"ok": True, "version": "v1", "overall_outcome": "success", "summary": {"message": "", "error": None}}

    def _exec(_spec: Any) -> Dict[str, Any]:
        return {
            "ok": False,
            "version": "v1",
            "overall_outcome": "failure",
            "summary": {"message": "", "error": "nav failed"},
        }

    out = run_planner_team_named_mission_checked(intent, inspect_checked=_insp, execute_checked=_exec)
    assert out["ok"] is False
    assert out["overall_outcome"] == "failure"
    assert out["execution"] is not None
    assert out["summary"]["error"] == "nav failed"
