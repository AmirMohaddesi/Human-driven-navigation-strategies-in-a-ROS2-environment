"""Unit tests for coordinator.assign_named_navigation (no ROS)."""

from __future__ import annotations

from typing import Any, Dict
from unittest.mock import MagicMock, patch

from multi_robot_mission_stack.coordinator import (
    assign_named_navigation,
    assign_named_parallel,
    assign_named_sequence,
    assign_team_named_mission,
    normalize_team_named_mission_spec,
    preflight_team_named_mission_spec,
    run_team_named_mission_spec,
    summarize_sequence_result,
)


def test_assign_succeeded() -> None:
    facade = MagicMock()
    facade.handle_command.return_value = {
        "status": "accepted",
        "goal_id": "g1",
        "nav_status": "submitted",
        "message": "",
    }

    def _wait(
        _f: Any,
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
        "multi_robot_mission_stack.coordinator.coordinator.MissionAgentFacade.with_ros",
        return_value=facade,
    ), patch(
        "multi_robot_mission_stack.coordinator.coordinator.wait_for_terminal_navigation_state",
        side_effect=_wait,
    ):
        out = assign_named_navigation("robot1", "base")

    assert out["outcome"] == "succeeded"
    assert out["goal_id"] == "g1"
    assert out["robot_id"] == "robot1"
    assert out["location_name"] == "base"
    facade.close.assert_called_once()


def test_assign_navigate_fails() -> None:
    facade = MagicMock()
    facade.handle_command.return_value = {
        "status": "failed",
        "nav_status": "unknown",
        "message": "denied",
    }

    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.MissionAgentFacade.with_ros",
        return_value=facade,
    ):
        out = assign_named_navigation("robot1", "base")

    assert out["outcome"] == "failed"
    assert out["goal_id"] == ""
    facade.close.assert_called_once()


def test_assign_named_sequence_two_legs() -> None:
    calls: list[int] = [0]

    def _fake_assign(rid: str, loc: str, **kwargs: Any) -> Dict[str, Any]:
        i = calls[0]
        calls[0] += 1
        return {
            "robot_id": rid,
            "location_name": loc,
            "goal_id": f"g{i}",
            "outcome": "succeeded",
            "status": "success",
            "nav_status": "succeeded",
            "message": "",
            "elapsed_sec": 0.1,
        }

    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.assign_named_navigation",
        side_effect=_fake_assign,
    ):
        out = assign_named_sequence(
            [
                {"robot_id": "robot1", "location_name": "base"},
                {"robot_id": "robot2", "location_name": "test_goal"},
            ],
            per_goal_timeout_sec=5.0,
            poll_interval_sec=0.1,
        )

    assert out["overall_outcome"] == "success"
    assert out["steps_run"] == 2
    assert out["succeeded_count"] == 2
    assert out["failed_count"] == 0
    assert out["stopped_early"] is False


def test_assign_named_sequence_stops_early() -> None:
    def _fake(rid: str, loc: str, **kwargs: Any) -> Dict[str, Any]:
        return {
            "robot_id": rid,
            "location_name": loc,
            "goal_id": "",
            "outcome": "failed",
            "status": "failed",
            "nav_status": "unknown",
            "message": "x",
            "elapsed_sec": 0.0,
        }

    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.assign_named_navigation",
        side_effect=_fake,
    ):
        out = assign_named_sequence(
            [
                {"robot_id": "robot1", "location_name": "base"},
                {"robot_id": "robot2", "location_name": "test_goal"},
            ],
            continue_on_failure=False,
        )

    assert out["overall_outcome"] == "failure"
    assert out["steps_run"] == 1
    assert out["stopped_early"] is True
    assert out["failed_count"] == 1


def test_assign_named_sequence_invalid_steps() -> None:
    out = assign_named_sequence(123)  # type: ignore[arg-type]
    assert out["overall_outcome"] == "failure"
    assert "error" in out


def test_summarize_sequence_result_full_success() -> None:
    summary = {
        "overall_outcome": "success",
        "total_steps": 2,
        "steps_run": 2,
        "succeeded_count": 2,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_failure": False,
        "steps": [
            {"index": 0, "step_outcome": "succeeded"},
            {"index": 1, "step_outcome": "succeeded"},
        ],
    }
    s = summarize_sequence_result(summary)
    assert s["ok"] is True
    assert s["overall_outcome"] == "success"
    assert s["mission_state"] == "completed"
    assert s["failed_step_indices"] == []
    assert s["succeeded_step_indices"] == [0, 1]
    assert s["first_failed_step_index"] is None
    assert s["last_step_index_run"] == 1
    assert s["message"] == "Mission completed successfully."


def test_summarize_sequence_result_stop_early_failure() -> None:
    summary = {
        "overall_outcome": "failure",
        "total_steps": 2,
        "steps_run": 1,
        "succeeded_count": 0,
        "failed_count": 1,
        "stopped_early": True,
        "continue_on_failure": False,
        "steps": [{"index": 0, "step_outcome": "failed"}],
    }
    s = summarize_sequence_result(summary)
    assert s["ok"] is False
    assert s["overall_outcome"] == "failure"
    assert s["mission_state"] == "failed_fast"
    assert s["failed_step_indices"] == [0]
    assert s["succeeded_step_indices"] == []
    assert s["first_failed_step_index"] == 0
    assert s["last_step_index_run"] == 0
    assert s["message"] == "Mission failed at step 1 and stopped early."


def test_summarize_sequence_result_continue_on_failure_partial() -> None:
    summary = {
        "overall_outcome": "failure",
        "total_steps": 2,
        "steps_run": 2,
        "succeeded_count": 1,
        "failed_count": 1,
        "stopped_early": False,
        "continue_on_failure": True,
        "steps": [
            {"index": 0, "step_outcome": "failed"},
            {"index": 1, "step_outcome": "succeeded"},
        ],
    }
    s = summarize_sequence_result(summary)
    assert s["ok"] is False
    assert s["mission_state"] == "partial_failure"
    assert s["failed_step_indices"] == [0]
    assert s["succeeded_step_indices"] == [1]
    assert s["message"] == "Mission completed with 1 failed step."


def test_summarize_sequence_result_malformed() -> None:
    s = summarize_sequence_result("not-a-dict")  # type: ignore[arg-type]
    assert s["ok"] is False
    assert s["overall_outcome"] == "error"
    assert s["mission_state"] == "invalid"
    assert s["message"] == "Invalid sequence summary."


def test_assign_named_parallel_two_success() -> None:
    calls: list[tuple[str, str]] = []

    def _fake(rid: str, loc: str, **kwargs: Any) -> Dict[str, Any]:
        calls.append((rid, loc))
        return {
            "robot_id": rid,
            "location_name": loc,
            "goal_id": f"g-{rid}",
            "outcome": "succeeded",
            "status": "success",
            "nav_status": "succeeded",
            "message": "",
            "elapsed_sec": 0.05,
        }

    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.assign_named_navigation",
        side_effect=_fake,
    ):
        out = assign_named_parallel(
            [
                {"robot_id": "robot1", "location_name": "base"},
                {"robot_id": "robot2", "location_name": "test_goal"},
            ],
            per_goal_timeout_sec=5.0,
            poll_interval_sec=0.1,
            max_workers=2,
        )

    assert out["overall_outcome"] == "success"
    assert out["steps_run"] == 2
    assert out["succeeded_count"] == 2
    assert out["failed_count"] == 0
    assert out["max_workers"] == 2
    assert [s["index"] for s in out["steps"]] == [0, 1]
    assert [s["robot_id"] for s in out["steps"]] == ["robot1", "robot2"]
    assert all(s["step_outcome"] == "succeeded" for s in out["steps"])
    assert len(calls) == 2


def test_assign_named_parallel_one_failure() -> None:
    def _fake(rid: str, loc: str, **kwargs: Any) -> Dict[str, Any]:
        ok = rid == "robot1"
        return {
            "robot_id": rid,
            "location_name": loc,
            "goal_id": "g1" if ok else "",
            "outcome": "succeeded" if ok else "failed",
            "status": "success" if ok else "failed",
            "nav_status": "succeeded" if ok else "unknown",
            "message": "" if ok else "nope",
            "elapsed_sec": 0.01,
        }

    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.assign_named_navigation",
        side_effect=_fake,
    ):
        out = assign_named_parallel(
            [
                {"robot_id": "robot1", "location_name": "base"},
                {"robot_id": "robot2", "location_name": "test_goal"},
            ],
            max_workers=2,
        )

    assert out["overall_outcome"] == "failure"
    assert out["succeeded_count"] == 1
    assert out["failed_count"] == 1
    assert out["steps"][0]["step_outcome"] == "succeeded"
    assert out["steps"][1]["step_outcome"] == "failed"


def test_assign_named_parallel_duplicate_robot_rejected() -> None:
    mock_nav = MagicMock()

    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.assign_named_navigation",
        mock_nav,
    ):
        out = assign_named_parallel(
            [
                {"robot_id": "robot1", "location_name": "a"},
                {"robot_id": "robot1", "location_name": "b"},
            ],
        )

    assert out["overall_outcome"] == "error"
    assert out["steps_run"] == 0
    assert out["steps"] == []
    assert "duplicate" in str(out.get("error", "")).lower()
    mock_nav.assert_not_called()


def test_assign_named_parallel_malformed_steps() -> None:
    out = assign_named_parallel("bad")  # type: ignore[arg-type]
    assert out["overall_outcome"] == "error"
    assert out["steps"] == []
    assert "error" in out


def test_assign_team_named_mission_sequence_dispatches() -> None:
    inner = {
        "overall_outcome": "success",
        "total_steps": 1,
        "steps_run": 1,
        "succeeded_count": 1,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_failure": True,
        "steps": [],
    }

    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.assign_named_sequence",
        return_value=inner,
    ) as mock_seq:
        out = assign_team_named_mission(
            [{"robot_id": "r1", "location_name": "a"}],
            "sequence",
            continue_on_failure=True,
            bridge_node_name="bn",
        )

    mock_seq.assert_called_once()
    _, kwargs = mock_seq.call_args
    assert kwargs["continue_on_failure"] is True
    assert kwargs["bridge_node_name"] == "bn"
    assert out["mode"] == "sequence"
    assert out["ok"] is True
    assert out["overall_outcome"] == "success"
    assert out["summary"] is inner


def test_assign_team_named_mission_parallel_dispatches() -> None:
    inner = {
        "overall_outcome": "failure",
        "total_steps": 2,
        "steps_run": 2,
        "succeeded_count": 1,
        "failed_count": 1,
        "max_workers": 2,
        "steps": [],
    }

    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.assign_named_parallel",
        return_value=inner,
    ) as mock_par:
        out = assign_team_named_mission(
            [
                {"robot_id": "robot1", "location_name": "base"},
                {"robot_id": "robot2", "location_name": "goal"},
            ],
            "parallel",
            continue_on_failure=True,
            max_workers=2,
        )

    mock_par.assert_called_once()
    _, kwargs = mock_par.call_args
    assert "continue_on_failure" not in kwargs
    assert kwargs["max_workers"] == 2
    assert out["mode"] == "parallel"
    assert out["ok"] is False
    assert out["overall_outcome"] == "failure"
    assert out["summary"] is inner


def test_assign_team_named_mission_invalid_mode() -> None:
    out = assign_team_named_mission([], mode="hybrid")
    assert out["mode"] == "hybrid"
    assert out["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["summary"] == {"error": "unsupported mode"}


def test_assign_team_named_mission_mode_normalization_parallel() -> None:
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.assign_named_parallel",
        return_value={"overall_outcome": "success", "steps": []},
    ):
        out = assign_team_named_mission([], mode=" Parallel ")

    assert out["mode"] == "parallel"
    assert out["ok"] is True


def test_normalize_team_named_mission_spec_success_trims() -> None:
    out = normalize_team_named_mission_spec(
        [
            {"robot_id": "  r1  ", "location_name": "  base  "},
            {"robot_id": "r2", "location_name": "goal", "extra": 1},
        ],
        mode=" Sequence ",
    )
    assert out["ok"] is True
    assert out["mode"] == "sequence"
    assert out["error"] is None
    assert out["steps"] == [
        {"robot_id": "r1", "location_name": "base"},
        {"robot_id": "r2", "location_name": "goal"},
    ]


def test_normalize_team_named_mission_spec_invalid_mode() -> None:
    out = normalize_team_named_mission_spec([], mode="hybrid")
    assert out["ok"] is False
    assert out["mode"] == "hybrid"
    assert out["steps"] == []
    assert out["error"] == "unsupported mode"


def test_normalize_team_named_mission_spec_non_list_steps() -> None:
    out = normalize_team_named_mission_spec("not-a-list", mode="sequence")  # type: ignore[arg-type]
    assert out["ok"] is False
    assert out["steps"] == []
    assert out["error"] == "steps must be a list"


def test_normalize_team_named_mission_spec_step_not_dict() -> None:
    out = normalize_team_named_mission_spec(["x"], mode="sequence")  # type: ignore[list-item]
    assert out["ok"] is False
    assert out["error"] == "step 0 must be a dict"

    out2 = normalize_team_named_mission_spec(
        [{"robot_id": "a", "location_name": "b"}, "x"],
        mode="sequence",
    )  # type: ignore[list-item]
    assert out2["ok"] is False
    assert out2["error"] == "step 1 must be a dict"


def test_normalize_team_named_mission_spec_empty_robot_id() -> None:
    out = normalize_team_named_mission_spec(
        [{"robot_id": "", "location_name": "b"}],
        mode="sequence",
    )
    assert out["ok"] is False
    assert out["error"] == "step 0 has empty robot_id"


def test_normalize_team_named_mission_spec_empty_location_name() -> None:
    out = normalize_team_named_mission_spec(
        [{"robot_id": "a", "location_name": "  "}],
        mode="sequence",
    )
    assert out["ok"] is False
    assert out["error"] == "step 0 has empty location_name"


def test_assign_team_named_mission_uses_normalized_before_dispatch() -> None:
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.assign_named_sequence",
        return_value={"overall_outcome": "success", "steps": []},
    ) as mock_seq:
        assign_team_named_mission(
            [{"robot_id": "  r1  ", "location_name": "  z  "}],
            " Sequence ",
            bridge_node_name="bn",
        )

    passed_steps = mock_seq.call_args[0][0]
    assert passed_steps == [{"robot_id": "r1", "location_name": "z"}]


def test_assign_team_named_mission_normalization_error_wrapper() -> None:
    mock_seq = MagicMock()
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.assign_named_sequence",
        mock_seq,
    ):
        out = assign_team_named_mission("bad", mode="sequence")  # type: ignore[arg-type]

    mock_seq.assert_not_called()
    assert out["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["summary"] == {"error": "steps must be a list"}


def test_run_team_named_mission_spec_sequence_dispatches() -> None:
    inner = {
        "mode": "sequence",
        "ok": True,
        "overall_outcome": "success",
        "summary": {"overall_outcome": "success"},
    }
    spec = {
        "mode": "sequence",
        "steps": [{"robot_id": "r1", "location_name": "a"}],
        "options": {
            "per_goal_timeout_sec": 30.0,
            "poll_interval_sec": 0.5,
            "continue_on_failure": True,
            "bridge_node_name": "bn",
        },
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.assign_team_named_mission",
        return_value=inner,
    ) as mock_assign:
        out = run_team_named_mission_spec(spec)

    mock_assign.assert_called_once_with(
        spec["steps"],
        "sequence",
        per_goal_timeout_sec=30.0,
        poll_interval_sec=0.5,
        continue_on_failure=True,
        max_workers=None,
        bridge_node_name="bn",
    )
    assert out is inner


def test_run_team_named_mission_spec_parallel_max_workers() -> None:
    inner = {"mode": "parallel", "ok": True, "overall_outcome": "success", "summary": {}}
    spec = {
        "mode": "parallel",
        "steps": [
            {"robot_id": "robot1", "location_name": "base"},
            {"robot_id": "robot2", "location_name": "goal"},
        ],
        "options": {"max_workers": 2, "per_goal_timeout_sec": 60.0},
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.assign_team_named_mission",
        return_value=inner,
    ) as mock_assign:
        out = run_team_named_mission_spec(spec)

    mock_assign.assert_called_once_with(
        spec["steps"],
        "parallel",
        per_goal_timeout_sec=60.0,
        poll_interval_sec=1.0,
        continue_on_failure=False,
        max_workers=2,
        bridge_node_name="mission_bridge_node",
    )
    assert out is inner


def test_run_team_named_mission_spec_ignores_unknown_option_keys() -> None:
    spec = {
        "mode": "sequence",
        "steps": [],
        "options": {"poll_interval_sec": 0.25, "extra": 99, "noise": {"x": 1}},
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.assign_team_named_mission",
        return_value={"ok": True, "mode": "sequence", "overall_outcome": "success", "summary": {}},
    ) as mock_assign:
        run_team_named_mission_spec(spec)

    _, kwargs = mock_assign.call_args
    assert kwargs["poll_interval_sec"] == 0.25
    assert "extra" not in kwargs
    assert "noise" not in kwargs


def test_run_team_named_mission_spec_not_dict() -> None:
    out = run_team_named_mission_spec("nope")  # type: ignore[arg-type]
    assert out["ok"] is False
    assert out["mode"] == "invalid"
    assert out["overall_outcome"] == "error"
    assert out["summary"] == {"error": "spec must be a dict"}


def test_run_team_named_mission_spec_options_not_dict() -> None:
    out = run_team_named_mission_spec({"mode": "sequence", "steps": [], "options": []})  # type: ignore[dict-item]
    assert out["ok"] is False
    assert out["mode"] == "invalid"
    assert out["overall_outcome"] == "error"
    assert out["summary"] == {"error": "options must be a dict"}


def test_preflight_team_named_mission_spec_sequence_success() -> None:
    out = preflight_team_named_mission_spec(
        {
            "mode": "sequence",
            "steps": [
                {"robot_id": "  r1  ", "location_name": "a"},
                {"robot_id": "r1", "location_name": "b"},
            ],
            "options": {"per_goal_timeout_sec": 99.0, "ignored": 1},
        }
    )
    assert out["ok"] is True
    assert out["mode"] == "sequence"
    assert out["overall_outcome"] == "success"
    assert out["step_count"] == 2
    assert out["parallel_robot_ids_unique"] is None
    assert out["normalized_steps"] == [
        {"robot_id": "r1", "location_name": "a"},
        {"robot_id": "r1", "location_name": "b"},
    ]
    assert out["options"]["per_goal_timeout_sec"] == 99.0
    assert "ignored" not in out["options"]
    assert out["summary"]["error"] is None
    assert "sequence" in out["summary"]["message"].lower()


def test_preflight_team_named_mission_spec_parallel_success() -> None:
    out = preflight_team_named_mission_spec(
        {
            "mode": "parallel",
            "steps": [
                {"robot_id": "robot1", "location_name": "base"},
                {"robot_id": "robot2", "location_name": "goal"},
            ],
            "options": {"max_workers": 2, "bridge_node_name": "bn"},
        }
    )
    assert out["ok"] is True
    assert out["parallel_robot_ids_unique"] is True
    assert out["summary"]["error"] is None
    assert "parallel" in out["summary"]["message"].lower()
    assert out["options"]["max_workers"] == 2
    assert out["options"]["bridge_node_name"] == "bn"


def test_preflight_team_named_mission_spec_parallel_duplicate_robot() -> None:
    out = preflight_team_named_mission_spec(
        {
            "mode": "parallel",
            "steps": [
                {"robot_id": " r1 ", "location_name": "a"},
                {"robot_id": "r1", "location_name": "b"},
            ],
        }
    )
    assert out["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["parallel_robot_ids_unique"] is False
    assert out["summary"]["error"] == "duplicate robot_id in parallel batch"
    assert len(out["normalized_steps"]) == 2


def test_preflight_team_named_mission_spec_not_dict() -> None:
    out = preflight_team_named_mission_spec("x")  # type: ignore[arg-type]
    assert out["ok"] is False
    assert out["mode"] == "invalid"
    assert out["normalized_steps"] == []
    assert out["step_count"] == 0
    assert out["parallel_robot_ids_unique"] is None
    assert out["options"] == {}
    assert out["summary"]["error"] == "spec must be a dict"


def test_preflight_team_named_mission_spec_options_not_dict() -> None:
    out = preflight_team_named_mission_spec({"mode": "sequence", "steps": [], "options": 1})  # type: ignore[dict-item]
    assert out["ok"] is False
    assert out["summary"]["error"] == "options must be a dict"
    assert out["options"] == {}


def test_preflight_team_named_mission_spec_normalization_error() -> None:
    out = preflight_team_named_mission_spec({"mode": "sequence", "steps": "bad"})  # type: ignore[dict-item]
    assert out["ok"] is False
    assert out["normalized_steps"] == []
    assert out["summary"]["error"] == "steps must be a list"
    assert out["mode"] == "sequence"
