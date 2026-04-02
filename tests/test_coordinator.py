"""Unit tests for coordinator.assign_named_navigation (no ROS)."""

from __future__ import annotations

from typing import Any, Dict
from unittest.mock import MagicMock, patch

from multi_robot_mission_stack.coordinator import (
    assign_named_navigation,
    cancel_navigation_via_facade,
    get_team_named_mission_api_manifest,
    assign_named_parallel,
    assign_named_sequence,
    assign_team_named_mission,
    execute_team_named_mission_spec,
    execute_team_named_mission_spec_checked,
    execute_team_named_mission_specs,
    execute_team_named_mission_specs_checked,
    inspect_team_named_mission_spec,
    inspect_team_named_mission_spec_checked,
    inspect_team_named_mission_specs,
    inspect_team_named_mission_specs_checked,
    normalize_team_named_mission_spec,
    plan_team_named_mission_api_call,
    resolve_team_named_mission_api_entrypoint,
    preflight_team_named_mission_spec,
    preflight_team_named_mission_specs,
    preview_team_named_mission_spec,
    preview_team_named_mission_specs,
    run_team_named_mission_spec,
    run_team_named_mission_specs,
    summarize_sequence_result,
    summarize_team_named_mission_result,
    summarize_team_named_mission_specs_result,
    validate_team_named_mission_api_manifest,
    validate_team_named_mission_api_request,
    validate_team_named_mission_execution_contract,
    validate_team_named_mission_inspection,
    validate_team_named_mission_specs_contract,
    validate_team_named_mission_specs_execution_contract,
    validate_team_named_mission_specs_inspection,
    validate_team_named_mission_specs_summary,
    validate_team_named_mission_spec_contract,
    validate_team_named_mission_summary,
)


def test_get_team_named_mission_api_manifest_success() -> None:
    m = get_team_named_mission_api_manifest()
    assert m["ok"] is True
    assert m["overall_outcome"] == "success"


def test_get_team_named_mission_api_manifest_version() -> None:
    m = get_team_named_mission_api_manifest()
    assert m["api_version"] == "v1"


def test_get_team_named_mission_api_manifest_single_entrypoints() -> None:
    ep = get_team_named_mission_api_manifest()["entrypoints"]["single_spec"]
    assert ep["inspect"] == "inspect_team_named_mission_spec"
    assert ep["inspect_checked"] == "inspect_team_named_mission_spec_checked"
    assert ep["execute"] == "execute_team_named_mission_spec"
    assert ep["execute_checked"] == "execute_team_named_mission_spec_checked"


def test_get_team_named_mission_api_manifest_batch_entrypoints() -> None:
    ep = get_team_named_mission_api_manifest()["entrypoints"]["batch_specs"]
    assert ep["inspect"] == "inspect_team_named_mission_specs"
    assert ep["inspect_checked"] == "inspect_team_named_mission_specs_checked"
    assert ep["execute"] == "execute_team_named_mission_specs"
    assert ep["execute_checked"] == "execute_team_named_mission_specs_checked"


def test_validate_team_named_mission_api_manifest_valid() -> None:
    v = validate_team_named_mission_api_manifest(get_team_named_mission_api_manifest())
    assert v["ok"] is True
    assert v["overall_outcome"] == "success"
    assert v["message"] == "Mission API manifest is internally consistent."


def test_validate_team_named_mission_api_manifest_bad_version() -> None:
    m = dict(get_team_named_mission_api_manifest())
    m["api_version"] = "v0"
    v = validate_team_named_mission_api_manifest(m)
    assert v["ok"] is False
    assert "api_version must be v1" in v["errors"]


def test_validate_team_named_mission_api_manifest_bad_single_entrypoint() -> None:
    m = dict(get_team_named_mission_api_manifest())
    ep = dict(m["entrypoints"])
    single = dict(ep["single_spec"])
    single["inspect"] = "wrong_name"
    ep["single_spec"] = single
    m["entrypoints"] = ep
    v = validate_team_named_mission_api_manifest(m)
    assert v["ok"] is False
    assert any("entrypoints.single_spec.inspect" in e for e in v["errors"])


def test_validate_team_named_mission_api_manifest_success_with_summary_error() -> None:
    m = dict(get_team_named_mission_api_manifest())
    m["summary"] = {**dict(m["summary"]), "error": "x"}
    v = validate_team_named_mission_api_manifest(m)
    assert v["ok"] is False
    assert "success requires summary.error null" in v["errors"]


def test_resolve_team_named_mission_api_entrypoint_single_inspect_checked() -> None:
    out = resolve_team_named_mission_api_entrypoint("single_spec", "inspect_checked")
    assert out["ok"] is True
    assert out["entrypoint"] == "inspect_team_named_mission_spec_checked"


def test_resolve_team_named_mission_api_entrypoint_batch_execute() -> None:
    out = resolve_team_named_mission_api_entrypoint("batch_specs", "execute")
    assert out["ok"] is True
    assert out["entrypoint"] == "execute_team_named_mission_specs"


def test_resolve_team_named_mission_api_entrypoint_unsupported_scope() -> None:
    out = resolve_team_named_mission_api_entrypoint("team", "inspect")
    assert out["ok"] is False
    assert out["entrypoint"] is None
    assert out["summary"]["error"] == "unsupported scope"


def test_resolve_team_named_mission_api_entrypoint_unsupported_operation() -> None:
    out = resolve_team_named_mission_api_entrypoint("single_spec", "run")
    assert out["ok"] is False
    assert out["entrypoint"] is None
    assert out["summary"]["error"] == "unsupported operation"


def test_validate_team_named_mission_api_request_single_execute_checked() -> None:
    out = validate_team_named_mission_api_request(
        {"scope": "single_spec", "operation": "execute_checked"},
    )
    assert out["ok"] is True
    assert out["entrypoint"] == "execute_team_named_mission_spec_checked"
    assert out["summary"]["message"] == "Mission API request validated successfully."


def test_validate_team_named_mission_api_request_batch_inspect() -> None:
    out = validate_team_named_mission_api_request(
        {"scope": "batch_specs", "operation": "inspect"},
    )
    assert out["ok"] is True
    assert out["entrypoint"] == "inspect_team_named_mission_specs"


def test_validate_team_named_mission_api_request_not_dict() -> None:
    out = validate_team_named_mission_api_request([])  # type: ignore[arg-type]
    assert out["ok"] is False
    assert out["scope"] == ""
    assert out["operation"] == ""
    assert out["summary"]["error"] == "request must be a dict"


def test_validate_team_named_mission_api_request_missing_scope() -> None:
    out = validate_team_named_mission_api_request({"operation": "inspect"})
    assert out["ok"] is False
    assert out["summary"]["error"] == "missing scope"


def test_validate_team_named_mission_api_request_missing_operation() -> None:
    out = validate_team_named_mission_api_request({"scope": "single_spec"})
    assert out["ok"] is False
    assert out["summary"]["error"] == "missing operation"


def test_validate_team_named_mission_api_request_unsupported_scope() -> None:
    out = validate_team_named_mission_api_request({"scope": "team", "operation": "inspect"})
    assert out["ok"] is False
    assert out["summary"]["error"] == "unsupported scope"


def test_validate_team_named_mission_api_request_unsupported_operation() -> None:
    out = validate_team_named_mission_api_request({"scope": "single_spec", "operation": "run"})
    assert out["ok"] is False
    assert out["summary"]["error"] == "unsupported operation"


def test_plan_team_named_mission_api_call_single_inspect_checked() -> None:
    out = plan_team_named_mission_api_call(
        {"scope": "single_spec", "operation": "inspect_checked"},
    )
    assert out["ok"] is True
    assert out["execution_kind"] == "offline"
    assert out["checked"] is True


def test_plan_team_named_mission_api_call_batch_execute() -> None:
    out = plan_team_named_mission_api_call({"scope": "batch_specs", "operation": "execute"})
    assert out["ok"] is True
    assert out["execution_kind"] == "live"
    assert out["checked"] is False


def test_plan_team_named_mission_api_call_invalid_request() -> None:
    out = plan_team_named_mission_api_call("bad")  # type: ignore[arg-type]
    assert out["ok"] is False
    assert out["execution_kind"] == "invalid"
    assert out["checked"] is None


def test_plan_team_named_mission_api_call_missing_operation_checked_null() -> None:
    out = plan_team_named_mission_api_call({"scope": "single_spec"})
    assert out["ok"] is False
    assert out["checked"] is None
    assert out["summary"]["error"] == "missing operation"


def test_cancel_navigation_via_facade_calls_handle_command_with_cancel_navigation_shape() -> None:
    facade = MagicMock()
    raw: Dict[str, Any] = {
        "status": "failure",
        "nav_status": "wrong_robot",
        "message": "Goal id belongs to another robot",
    }
    facade.handle_command.return_value = raw

    out = cancel_navigation_via_facade(facade, "robot2", "goal-from-robot1")

    facade.handle_command.assert_called_once()
    cmd = facade.handle_command.call_args[0][0]
    assert cmd == {
        "type": "cancel",
        "target": "navigation",
        "robot_id": "robot2",
        "goal_id": "goal-from-robot1",
    }
    assert out is raw


def test_cancel_navigation_via_facade_strips_robot_id_and_goal_id() -> None:
    facade = MagicMock()
    ret = {"status": "accepted", "nav_status": "cancelling"}
    facade.handle_command.return_value = ret

    out = cancel_navigation_via_facade(facade, "  robot1  ", "  gid-7  ")

    cmd = facade.handle_command.call_args[0][0]
    assert cmd["robot_id"] == "robot1"
    assert cmd["goal_id"] == "gid-7"
    assert out is ret


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


def test_preview_team_named_mission_spec_sequence_success() -> None:
    out = preview_team_named_mission_spec(
        {
            "mode": "sequence",
            "steps": [
                {"robot_id": "robot1", "location_name": "base"},
                {"robot_id": "robot2", "location_name": "goal"},
            ],
            "options": {"continue_on_failure": True},
        }
    )
    assert out["ok"] is True
    assert out["overall_outcome"] == "success"
    assert out["step_count"] == 2
    assert out["lines"][:2] == ["Mode: sequence", "Steps: 2"]
    assert out["lines"][2] == "Step 0: robot1 -> base"
    assert out["lines"][3] == "Step 1: robot2 -> goal"
    assert out["lines"][-1] == "Continue on failure: true"
    assert out["summary"]["error"] is None
    assert "sequence" in out["summary"]["message"].lower()


def test_preview_team_named_mission_spec_parallel_max_workers() -> None:
    out = preview_team_named_mission_spec(
        {
            "mode": "parallel",
            "steps": [
                {"robot_id": "robot1", "location_name": "base"},
                {"robot_id": "robot2", "location_name": "test_goal"},
            ],
            "options": {"max_workers": 3},
        }
    )
    assert out["ok"] is True
    assert out["lines"][-1] == "Max workers: 3"
    assert "Continue on failure" not in "\n".join(out["lines"])
    assert "parallel" in out["summary"]["message"].lower()


def test_preview_team_named_mission_spec_preflight_failure() -> None:
    out = preview_team_named_mission_spec("nope")  # type: ignore[arg-type]
    assert out["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["lines"] == ["Mode: invalid", "Preflight failed: spec must be a dict"]
    assert out["summary"]["error"] == "spec must be a dict"


def test_preview_team_named_mission_spec_lines_normalized_order() -> None:
    out = preview_team_named_mission_spec(
        {
            "mode": "sequence",
            "steps": [
                {"robot_id": "  z  ", "location_name": " first "},
                {"robot_id": "a", "location_name": "second"},
            ],
        }
    )
    assert out["lines"][2:4] == [
        "Step 0: z -> first",
        "Step 1: a -> second",
    ]


def test_preflight_team_named_mission_specs_two_valid() -> None:
    specs = [
        {
            "mode": "sequence",
            "steps": [{"robot_id": "r1", "location_name": "a"}],
        },
        {
            "mode": "parallel",
            "steps": [
                {"robot_id": "r1", "location_name": "b"},
                {"robot_id": "r2", "location_name": "c"},
            ],
        },
    ]
    out = preflight_team_named_mission_specs(specs)
    assert out["ok"] is True
    assert out["overall_outcome"] == "success"
    assert out["total_specs"] == 2
    assert out["ok_count"] == 2
    assert out["error_count"] == 0
    assert len(out["results"]) == 2
    assert out["results"][0]["index"] == 0 and out["results"][0]["ok"] is True
    assert out["results"][1]["index"] == 1 and out["results"][1]["ok"] is True
    assert out["summary"]["message"] == "Preflight passed for 2 mission specs."
    assert out["summary"]["error"] is None
    assert "normalized_steps" not in out["results"][0]


def test_preflight_team_named_mission_specs_mixed() -> None:
    specs = [
        {"mode": "sequence", "steps": [{"robot_id": "r1", "location_name": "a"}]},
        "bad",  # type: ignore[list-item]
    ]
    out = preflight_team_named_mission_specs(specs)
    assert out["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["ok_count"] == 1
    assert out["error_count"] == 1
    assert out["results"][0]["ok"] is True
    assert out["results"][1]["ok"] is False
    assert out["results"][1]["summary"]["error"] == "spec must be a dict"
    assert out["summary"]["message"] == "Preflight completed with 1 invalid mission spec."


def test_preflight_team_named_mission_specs_not_list() -> None:
    out = preflight_team_named_mission_specs({})  # type: ignore[arg-type]
    assert out["ok"] is False
    assert out["total_specs"] == 0
    assert out["results"] == []
    assert out["summary"]["error"] == "specs must be a list"


def test_preflight_team_named_mission_specs_order_preserved() -> None:
    specs = [
        "x",  # type: ignore[list-item]
        {"mode": "sequence", "steps": [{"robot_id": "r1", "location_name": "a"}]},
    ]
    out = preflight_team_named_mission_specs(specs)
    assert [r["index"] for r in out["results"]] == [0, 1]
    assert out["results"][0]["ok"] is False
    assert out["results"][1]["ok"] is True
    assert out["results"][0]["mode"] == "invalid"


def test_preview_team_named_mission_specs_two_valid() -> None:
    specs = [
        {"mode": "sequence", "steps": [{"robot_id": "robot1", "location_name": "base"}]},
        {
            "mode": "parallel",
            "steps": [
                {"robot_id": "r1", "location_name": "b"},
                {"robot_id": "r2", "location_name": "c"},
            ],
        },
    ]
    out = preview_team_named_mission_specs(specs)
    assert out["ok"] is True
    assert out["overall_outcome"] == "success"
    assert out["total_specs"] == 2
    assert out["ok_count"] == 2
    assert out["error_count"] == 0
    assert out["summary"]["message"] == "Preview generated for 2 mission specs."
    assert out["results"][0]["lines"][0] == "Mode: sequence"
    assert out["results"][0]["lines"][2] == "Step 0: robot1 -> base"
    assert "lines" in out["results"][0]


def test_preview_team_named_mission_specs_mixed() -> None:
    specs = [
        {"mode": "sequence", "steps": [{"robot_id": "r1", "location_name": "a"}]},
        "bad",  # type: ignore[list-item]
    ]
    out = preview_team_named_mission_specs(specs)
    assert out["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["ok_count"] == 1
    assert out["error_count"] == 1
    assert out["results"][1]["lines"] == ["Mode: invalid", "Preflight failed: spec must be a dict"]
    assert out["summary"]["message"] == "Preview completed with 1 invalid mission spec."


def test_preview_team_named_mission_specs_not_list() -> None:
    out = preview_team_named_mission_specs(())  # type: ignore[arg-type]
    assert out["ok"] is False
    assert out["total_specs"] == 0
    assert out["results"] == []
    assert out["summary"]["error"] == "specs must be a list"


def test_preview_team_named_mission_specs_order_preserved() -> None:
    specs = [
        "x",  # type: ignore[list-item]
        {"mode": "sequence", "steps": [{"robot_id": "r1", "location_name": "a"}]},
    ]
    out = preview_team_named_mission_specs(specs)
    assert [r["index"] for r in out["results"]] == [0, 1]
    assert out["results"][0]["ok"] is False
    assert out["results"][1]["ok"] is True


def test_run_team_named_mission_specs_two_success() -> None:
    ok_row = {"ok": True, "mode": "sequence", "overall_outcome": "success", "summary": {"x": 1}}

    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_spec",
        return_value=ok_row,
    ) as mock_run:
        out = run_team_named_mission_specs([{}, {}])

    assert mock_run.call_count == 2
    assert out["ok"] is True
    assert out["overall_outcome"] == "success"
    assert out["total_specs"] == 2
    assert out["specs_run"] == 2
    assert out["succeeded_count"] == 2
    assert out["failed_count"] == 0
    assert out["stopped_early"] is False
    assert out["summary"]["message"] == "Executed 2 mission specs successfully."


def test_run_team_named_mission_specs_stops_after_first_failure() -> None:
    fail_row = {"ok": False, "mode": "sequence", "overall_outcome": "failure", "summary": {}}
    ok_row = {"ok": True, "mode": "sequence", "overall_outcome": "success", "summary": {}}

    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_spec",
        side_effect=[fail_row, ok_row],
    ) as mock_run:
        out = run_team_named_mission_specs([{}, {}], continue_on_batch_failure=False)

    mock_run.assert_called_once()
    assert out["ok"] is False
    assert out["overall_outcome"] == "failure"
    assert out["specs_run"] == 1
    assert out["failed_count"] == 1
    assert out["stopped_early"] is True
    assert out["summary"]["message"] == "Execution stopped after 1 failed mission spec."


def test_run_team_named_mission_specs_continue_on_batch_failure() -> None:
    fail_row = {"ok": False, "mode": "sequence", "overall_outcome": "failure", "summary": {}}
    ok_row = {"ok": True, "mode": "parallel", "overall_outcome": "success", "summary": {}}

    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_spec",
        side_effect=[fail_row, ok_row, fail_row],
    ) as mock_run:
        out = run_team_named_mission_specs([{}, {}, {}], continue_on_batch_failure=True)

    assert mock_run.call_count == 3
    assert out["specs_run"] == 3
    assert out["succeeded_count"] == 1
    assert out["failed_count"] == 2
    assert out["stopped_early"] is False
    assert out["summary"]["message"] == "Execution completed with 2 failed mission specs."


def test_run_team_named_mission_specs_not_list() -> None:
    out = run_team_named_mission_specs("nope")  # type: ignore[arg-type]
    assert out["overall_outcome"] == "error"
    assert out["summary"]["error"] == "specs must be a list"
    assert out["results"] == []


def test_run_team_named_mission_specs_order_preserved() -> None:
    calls: list[int] = []

    def _fake(spec: Any) -> Dict[str, Any]:
        calls.append(id(spec))
        return {"ok": True, "mode": "sequence", "overall_outcome": "success", "summary": {}}

    s1 = {"a": 1}
    s2 = {"b": 2}
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_spec",
        side_effect=_fake,
    ):
        out = run_team_named_mission_specs([s1, s2])

    assert calls == [id(s1), id(s2)]
    assert [r["index"] for r in out["results"]] == [0, 1]


def test_summarize_team_named_mission_specs_result_full_success() -> None:
    batch = {
        "ok": True,
        "overall_outcome": "success",
        "total_specs": 2,
        "specs_run": 2,
        "succeeded_count": 2,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_batch_failure": False,
        "results": [
            {"index": 0, "ok": True, "mode": "sequence", "overall_outcome": "success", "summary": {}},
            {"index": 1, "ok": True, "mode": "parallel", "overall_outcome": "success", "summary": {}},
        ],
        "summary": {},
    }
    s = summarize_team_named_mission_specs_result(batch)
    assert s["ok"] is True
    assert s["overall_outcome"] == "success"
    assert s["mission_state"] == "completed"
    assert s["failed_spec_indices"] == []
    assert s["succeeded_spec_indices"] == [0, 1]
    assert s["first_failed_spec_index"] is None
    assert s["last_spec_index_run"] == 1
    assert s["message"] == "Batch execution completed successfully."


def test_summarize_team_named_mission_specs_result_failed_fast() -> None:
    batch = {
        "ok": False,
        "overall_outcome": "failure",
        "total_specs": 2,
        "specs_run": 1,
        "succeeded_count": 0,
        "failed_count": 1,
        "stopped_early": True,
        "continue_on_batch_failure": False,
        "results": [{"index": 0, "ok": False, "mode": "sequence", "overall_outcome": "failure", "summary": {}}],
        "summary": {},
    }
    s = summarize_team_named_mission_specs_result(batch)
    assert s["mission_state"] == "failed_fast"
    assert s["message"] == "Batch execution failed at spec 1 and stopped early."


def test_summarize_team_named_mission_specs_result_partial_failure() -> None:
    batch = {
        "ok": False,
        "overall_outcome": "failure",
        "total_specs": 3,
        "specs_run": 3,
        "succeeded_count": 1,
        "failed_count": 2,
        "stopped_early": False,
        "continue_on_batch_failure": True,
        "results": [
            {"index": 0, "ok": True, "mode": "sequence", "overall_outcome": "success", "summary": {}},
            {"index": 1, "ok": False, "mode": "sequence", "overall_outcome": "failure", "summary": {}},
            {"index": 2, "ok": False, "mode": "sequence", "overall_outcome": "failure", "summary": {}},
        ],
        "summary": {},
    }
    s = summarize_team_named_mission_specs_result(batch)
    assert s["mission_state"] == "partial_failure"
    assert s["failed_spec_indices"] == [1, 2]
    assert s["message"] == "Batch execution completed with 2 failed specs."


def test_summarize_team_named_mission_specs_result_malformed() -> None:
    s = summarize_team_named_mission_specs_result("x")  # type: ignore[arg-type]
    assert s["ok"] is False
    assert s["overall_outcome"] == "error"
    assert s["mission_state"] == "invalid"
    assert s["message"] == "Invalid batch execution result."


def test_summarize_team_named_mission_result_sequence_success() -> None:
    inner = {
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
    r = summarize_team_named_mission_result(
        {"ok": True, "mode": "sequence", "overall_outcome": "success", "summary": inner}
    )
    assert r["ok"] is True
    assert r["mode"] == "sequence"
    assert r["mission_state"] == "completed"
    assert r["stopped_early"] is False
    assert r["message"] == "Sequence mission completed successfully."


def test_summarize_team_named_mission_result_sequence_failed_fast() -> None:
    inner = {
        "overall_outcome": "failure",
        "total_steps": 2,
        "steps_run": 1,
        "succeeded_count": 0,
        "failed_count": 1,
        "stopped_early": True,
        "continue_on_failure": False,
        "steps": [{"index": 0, "step_outcome": "failed"}],
    }
    r = summarize_team_named_mission_result(
        {"ok": False, "mode": "sequence", "overall_outcome": "failure", "summary": inner}
    )
    assert r["mission_state"] == "failed_fast"
    assert r["message"] == "Sequence mission failed at step 1 and stopped early."


def test_summarize_team_named_mission_result_parallel_partial() -> None:
    inner = {
        "overall_outcome": "failure",
        "total_steps": 2,
        "steps_run": 2,
        "succeeded_count": 1,
        "failed_count": 1,
        "max_workers": 2,
        "steps": [
            {"index": 0, "step_outcome": "succeeded"},
            {"index": 1, "step_outcome": "failed"},
        ],
    }
    r = summarize_team_named_mission_result(
        {"ok": False, "mode": "parallel", "overall_outcome": "failure", "summary": inner}
    )
    assert r["mode"] == "parallel"
    assert r["mission_state"] == "partial_failure"
    assert r["stopped_early"] is None
    assert r["continue_on_failure"] is None
    assert r["failed_step_indices"] == [1]
    assert r["message"] == "Parallel mission completed with 1 failed step."


def test_summarize_team_named_mission_result_malformed() -> None:
    r = summarize_team_named_mission_result({"mode": "sequence", "overall_outcome": "error", "summary": {}})
    assert r["overall_outcome"] == "error"
    assert r["mission_state"] == "invalid"
    assert r["mode"] == "invalid"
    assert r["message"] == "Invalid team mission result."


def test_inspect_team_named_mission_spec_sequence_success() -> None:
    spec = {
        "mode": "sequence",
        "steps": [
            {"robot_id": "robot1", "location_name": "base"},
            {"robot_id": "robot2", "location_name": "goal"},
        ],
    }
    out = inspect_team_named_mission_spec(spec)
    assert out["ok"] is True
    assert out["overall_outcome"] == "success"
    assert out["mode"] == "sequence"
    assert out["step_count"] == 2
    assert out["preflight"]["ok"] is True
    assert out["preview"]["ok"] is True
    assert out["summary"]["message"] == "Mission spec inspection completed successfully."
    assert out["summary"]["error"] is None


def test_inspect_team_named_mission_spec_parallel_success() -> None:
    spec = {
        "mode": "parallel",
        "steps": [
            {"robot_id": "robot1", "location_name": "a"},
            {"robot_id": "robot2", "location_name": "b"},
        ],
    }
    out = inspect_team_named_mission_spec(spec)
    assert out["ok"] is True
    assert out["mode"] == "parallel"
    assert out["preflight"]["overall_outcome"] == "success"
    assert out["preview"]["overall_outcome"] == "success"


def test_inspect_team_named_mission_spec_preflight_failure() -> None:
    spec = {
        "mode": "parallel",
        "steps": [
            {"robot_id": "r1", "location_name": "a"},
            {"robot_id": "r1", "location_name": "b"},
        ],
    }
    out = inspect_team_named_mission_spec(spec)
    assert out["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["preflight"]["ok"] is False
    assert "duplicate" in str(out["summary"]["error"]).lower()
    assert out["summary"]["message"] == "Mission spec inspection failed."
    assert len(out["preview"]["lines"]) >= 2


def test_inspect_team_named_mission_spec_preview_lines_order() -> None:
    spec = {
        "mode": "sequence",
        "steps": [
            {"robot_id": "z", "location_name": "first"},
            {"robot_id": "a", "location_name": "second"},
        ],
    }
    out = inspect_team_named_mission_spec(spec)
    expected = preview_team_named_mission_spec(spec)["lines"]
    assert out["preview"]["lines"] == expected


def test_validate_team_named_mission_inspection_valid() -> None:
    spec = {
        "mode": "sequence",
        "steps": [
            {"robot_id": "robot1", "location_name": "base"},
            {"robot_id": "robot2", "location_name": "goal"},
        ],
    }
    ins = inspect_team_named_mission_spec(spec)
    v = validate_team_named_mission_inspection(ins)
    assert v["ok"] is True
    assert v["overall_outcome"] == "success"
    assert v["message"] == "Mission inspection is internally consistent."


def test_validate_team_named_mission_inspection_preview_success_with_error() -> None:
    inv: Dict[str, Any] = {
        "ok": True,
        "mode": "sequence",
        "overall_outcome": "success",
        "step_count": 1,
        "preflight": {
            "ok": True,
            "overall_outcome": "success",
            "summary": {"message": "", "error": None},
        },
        "preview": {
            "ok": True,
            "overall_outcome": "success",
            "lines": ["Mode: sequence", "Steps: 1", "Step 0: r -> g"],
            "summary": {"message": "", "error": "inconsistent"},
        },
        "summary": {"message": "", "error": None},
    }
    v = validate_team_named_mission_inspection(inv)
    assert v["ok"] is False
    assert "preview success requires summary.error null" in v["errors"]


def test_inspect_team_named_mission_spec_checked_sequence_success() -> None:
    spec = {
        "mode": "sequence",
        "steps": [
            {"robot_id": "robot1", "location_name": "base"},
            {"robot_id": "robot2", "location_name": "goal"},
        ],
    }
    out = inspect_team_named_mission_spec_checked(spec)
    assert out["ok"] is True
    assert out["overall_outcome"] == "success"
    assert out["mode"] == "sequence"
    assert out["validation"]["ok"] is True
    assert out["summary"]["message"] == "Mission spec inspected and validated successfully."
    assert out["summary"]["error"] is None


def test_inspect_team_named_mission_spec_checked_malformed_inspection() -> None:
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.inspect_team_named_mission_spec",
        return_value={},
    ):
        out = inspect_team_named_mission_spec_checked({})

    assert out["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["validation"]["ok"] is False
    assert out["summary"]["error"] == "Mission inspection is invalid."


def test_validate_team_named_mission_specs_inspection_valid() -> None:
    specs = [
        {"mode": "sequence", "steps": [{"robot_id": "r1", "location_name": "a"}]},
        {
            "mode": "parallel",
            "steps": [
                {"robot_id": "r1", "location_name": "b"},
                {"robot_id": "r2", "location_name": "c"},
            ],
        },
    ]
    batch = inspect_team_named_mission_specs(specs)
    v = validate_team_named_mission_specs_inspection(batch)
    assert v["ok"] is True
    assert v["message"] == "Batch mission inspection is internally consistent."


def test_validate_team_named_mission_specs_inspection_success_with_errors() -> None:
    row = {
        "index": 0,
        "ok": True,
        "mode": "sequence",
        "overall_outcome": "success",
        "step_count": 1,
        "preflight": {},
        "preview": {},
        "summary": {},
    }
    inv: Dict[str, Any] = {
        "ok": True,
        "overall_outcome": "success",
        "total_specs": 2,
        "ok_count": 1,
        "error_count": 1,
        "results": [row, {**row, "index": 1, "ok": False, "overall_outcome": "error", "mode": "invalid", "step_count": 0}],
        "summary": {"message": "", "error": None},
    }
    v = validate_team_named_mission_specs_inspection(inv)
    assert v["ok"] is False
    assert "error_count>0 is incompatible with overall_outcome success" in v["errors"]


def test_inspect_team_named_mission_specs_checked_two_valid() -> None:
    specs = [
        {"mode": "sequence", "steps": [{"robot_id": "r1", "location_name": "a"}]},
        {
            "mode": "parallel",
            "steps": [
                {"robot_id": "r1", "location_name": "b"},
                {"robot_id": "r2", "location_name": "c"},
            ],
        },
    ]
    out = inspect_team_named_mission_specs_checked(specs)
    assert out["ok"] is True
    assert out["overall_outcome"] == "success"
    assert out["validation"]["ok"] is True
    assert out["summary"]["message"] == "Batch mission specs inspected and validated successfully."
    assert out["summary"]["error"] is None
    assert out["inspection"]["total_specs"] == 2


def test_inspect_team_named_mission_specs_checked_malformed_inspection() -> None:
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.inspect_team_named_mission_specs",
        return_value={},
    ):
        out = inspect_team_named_mission_specs_checked([])

    assert out["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["validation"]["ok"] is False
    assert out["summary"]["error"] == "Batch mission inspection is invalid."


def test_inspect_team_named_mission_specs_two_valid() -> None:
    specs = [
        {"mode": "sequence", "steps": [{"robot_id": "r1", "location_name": "a"}]},
        {
            "mode": "parallel",
            "steps": [
                {"robot_id": "r1", "location_name": "b"},
                {"robot_id": "r2", "location_name": "c"},
            ],
        },
    ]
    out = inspect_team_named_mission_specs(specs)
    assert out["ok"] is True
    assert out["overall_outcome"] == "success"
    assert out["total_specs"] == 2
    assert out["ok_count"] == 2
    assert out["error_count"] == 0
    assert len(out["results"]) == 2
    assert out["summary"]["message"] == "Inspection completed successfully for 2 mission specs."
    assert out["results"][0]["preflight"]["ok"] is True
    assert out["results"][1]["preview"]["lines"][0] == "Mode: parallel"


def test_inspect_team_named_mission_specs_mixed() -> None:
    specs = [
        {"mode": "sequence", "steps": [{"robot_id": "r1", "location_name": "a"}]},
        {
            "mode": "parallel",
            "steps": [
                {"robot_id": "r1", "location_name": "x"},
                {"robot_id": "r1", "location_name": "y"},
            ],
        },
    ]
    out = inspect_team_named_mission_specs(specs)
    assert out["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["ok_count"] == 1
    assert out["error_count"] == 1
    assert out["results"][0]["ok"] is True
    assert out["results"][1]["ok"] is False
    assert out["summary"]["message"] == "Inspection completed with 1 invalid mission spec."


def test_inspect_team_named_mission_specs_not_list() -> None:
    out = inspect_team_named_mission_specs({})  # type: ignore[arg-type]
    assert out["ok"] is False
    assert out["total_specs"] == 0
    assert out["results"] == []
    assert out["summary"]["error"] == "specs must be a list"


def test_execute_team_named_mission_spec_sequence_success() -> None:
    run = {
        "ok": True,
        "mode": "sequence",
        "overall_outcome": "success",
        "summary": {"overall_outcome": "success", "total_steps": 1, "steps_run": 1, "steps": []},
    }
    ex = {
        "ok": True,
        "mode": "sequence",
        "overall_outcome": "success",
        "mission_state": "completed",
        "total_steps": 1,
        "steps_run": 1,
        "succeeded_count": 1,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_failure": False,
        "failed_step_indices": [],
        "succeeded_step_indices": [],
        "first_failed_step_index": None,
        "last_step_index_run": None,
        "message": "Sequence mission completed successfully.",
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_spec",
        return_value=run,
    ), patch(
        "multi_robot_mission_stack.coordinator.coordinator.summarize_team_named_mission_result",
        return_value=ex,
    ):
        out = execute_team_named_mission_spec({})

    assert out["ok"] is True
    assert out["overall_outcome"] == "success"
    assert out["run_result"]["ok"] is True
    assert out["execution_summary"]["mission_state"] == "completed"
    assert out["summary"]["message"] == "Mission spec executed successfully."
    assert out["summary"]["error"] is None


def test_execute_team_named_mission_spec_sequence_failed_fast() -> None:
    run = {
        "ok": False,
        "mode": "sequence",
        "overall_outcome": "failure",
        "summary": {},
    }
    ex = {
        "ok": False,
        "mode": "sequence",
        "overall_outcome": "failure",
        "mission_state": "failed_fast",
        "total_steps": 2,
        "steps_run": 1,
        "succeeded_count": 0,
        "failed_count": 1,
        "stopped_early": True,
        "continue_on_failure": False,
        "failed_step_indices": [0],
        "succeeded_step_indices": [],
        "first_failed_step_index": 0,
        "last_step_index_run": 0,
        "message": "Sequence mission failed at step 1 and stopped early.",
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_spec",
        return_value=run,
    ), patch(
        "multi_robot_mission_stack.coordinator.coordinator.summarize_team_named_mission_result",
        return_value=ex,
    ):
        out = execute_team_named_mission_spec({})

    assert out["ok"] is False
    assert out["overall_outcome"] == "failure"
    assert out["execution_summary"]["mission_state"] == "failed_fast"
    assert out["summary"]["error"] == "Sequence mission failed at step 1 and stopped early."


def test_execute_team_named_mission_spec_parallel_partial() -> None:
    run = {
        "ok": False,
        "mode": "parallel",
        "overall_outcome": "failure",
        "summary": {},
    }
    ex = {
        "ok": False,
        "mode": "parallel",
        "overall_outcome": "failure",
        "mission_state": "partial_failure",
        "total_steps": 2,
        "steps_run": 2,
        "succeeded_count": 1,
        "failed_count": 1,
        "stopped_early": None,
        "continue_on_failure": None,
        "failed_step_indices": [1],
        "succeeded_step_indices": [0],
        "first_failed_step_index": 1,
        "last_step_index_run": 1,
        "message": "Parallel mission completed with 1 failed step.",
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_spec",
        return_value=run,
    ), patch(
        "multi_robot_mission_stack.coordinator.coordinator.summarize_team_named_mission_result",
        return_value=ex,
    ):
        out = execute_team_named_mission_spec({})

    assert out["mode"] == "parallel"
    assert out["execution_summary"]["mission_state"] == "partial_failure"
    assert out["summary"]["error"] == "Parallel mission completed with 1 failed step."


def test_execute_team_named_mission_specs_two_success() -> None:
    run = {
        "ok": True,
        "overall_outcome": "success",
        "total_specs": 2,
        "specs_run": 2,
        "succeeded_count": 2,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_batch_failure": False,
        "results": [],
        "summary": {"message": "ok", "error": None},
    }
    ex = {
        "ok": True,
        "overall_outcome": "success",
        "mission_state": "completed",
        "total_specs": 2,
        "specs_run": 2,
        "succeeded_count": 2,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_batch_failure": False,
        "failed_spec_indices": [],
        "succeeded_spec_indices": [0, 1],
        "first_failed_spec_index": None,
        "last_spec_index_run": 1,
        "message": "Batch execution completed successfully.",
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_specs",
        return_value=run,
    ), patch(
        "multi_robot_mission_stack.coordinator.coordinator.summarize_team_named_mission_specs_result",
        return_value=ex,
    ):
        out = execute_team_named_mission_specs([{}, {}])

    assert out["ok"] is True
    assert out["overall_outcome"] == "success"
    assert out["summary"]["message"] == "Batch mission specs executed successfully."
    assert out["run_result"]["total_specs"] == 2


def test_execute_team_named_mission_specs_stop_early() -> None:
    run = {
        "ok": False,
        "overall_outcome": "failure",
        "total_specs": 2,
        "specs_run": 1,
        "succeeded_count": 0,
        "failed_count": 1,
        "stopped_early": True,
        "continue_on_batch_failure": False,
        "results": [{"index": 0, "ok": False, "mode": "sequence", "overall_outcome": "failure", "summary": {}}],
        "summary": {"message": "stopped", "error": None},
    }
    ex = {
        "ok": False,
        "overall_outcome": "failure",
        "mission_state": "failed_fast",
        "total_specs": 2,
        "specs_run": 1,
        "succeeded_count": 0,
        "failed_count": 1,
        "stopped_early": True,
        "continue_on_batch_failure": False,
        "failed_spec_indices": [0],
        "succeeded_spec_indices": [],
        "first_failed_spec_index": 0,
        "last_spec_index_run": 0,
        "message": "Batch execution failed at spec 1 and stopped early.",
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_specs",
        return_value=run,
    ), patch(
        "multi_robot_mission_stack.coordinator.coordinator.summarize_team_named_mission_specs_result",
        return_value=ex,
    ):
        out = execute_team_named_mission_specs([{}, {}])

    assert out["ok"] is False
    assert out["overall_outcome"] == "failure"
    assert out["summary"]["message"] == ex["message"]
    assert out["execution_summary"]["mission_state"] == "failed_fast"


def test_execute_team_named_mission_specs_continue_on_failure() -> None:
    run = {
        "ok": False,
        "overall_outcome": "failure",
        "total_specs": 3,
        "specs_run": 3,
        "succeeded_count": 1,
        "failed_count": 2,
        "stopped_early": False,
        "continue_on_batch_failure": True,
        "results": [],
        "summary": {"message": "done", "error": None},
    }
    ex = {
        "ok": False,
        "overall_outcome": "failure",
        "mission_state": "partial_failure",
        "total_specs": 3,
        "specs_run": 3,
        "succeeded_count": 1,
        "failed_count": 2,
        "stopped_early": False,
        "continue_on_batch_failure": True,
        "failed_spec_indices": [1, 2],
        "succeeded_spec_indices": [0],
        "first_failed_spec_index": 1,
        "last_spec_index_run": 2,
        "message": "Batch execution completed with 2 failed specs.",
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_specs",
        return_value=run,
    ) as mock_run, patch(
        "multi_robot_mission_stack.coordinator.coordinator.summarize_team_named_mission_specs_result",
        return_value=ex,
    ):
        execute_team_named_mission_specs([{}, {}, {}], continue_on_batch_failure=True)

    mock_run.assert_called_once()
    assert mock_run.call_args.kwargs.get("continue_on_batch_failure") is True


def test_validate_team_named_mission_summary_valid() -> None:
    s = {
        "ok": True,
        "mode": "sequence",
        "overall_outcome": "success",
        "mission_state": "completed",
        "total_steps": 2,
        "steps_run": 2,
        "succeeded_count": 2,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_failure": False,
        "failed_step_indices": [],
        "succeeded_step_indices": [0, 1],
        "first_failed_step_index": None,
        "last_step_index_run": 1,
        "message": "x",
    }
    v = validate_team_named_mission_summary(s)
    assert v["ok"] is True
    assert v["overall_outcome"] == "success"
    assert "consistent" in v["message"].lower()


def test_validate_team_named_mission_summary_rejects_success_with_failures() -> None:
    s = {
        "ok": True,
        "mode": "sequence",
        "overall_outcome": "success",
        "mission_state": "completed",
        "total_steps": 2,
        "steps_run": 2,
        "succeeded_count": 1,
        "failed_count": 1,
        "stopped_early": False,
        "continue_on_failure": False,
        "failed_step_indices": [1],
        "succeeded_step_indices": [0],
        "first_failed_step_index": 1,
        "last_step_index_run": 1,
        "message": "x",
    }
    v = validate_team_named_mission_summary(s)
    assert v["ok"] is False
    assert v["errors"]


def test_validate_team_named_mission_specs_summary_valid() -> None:
    s = {
        "ok": True,
        "overall_outcome": "success",
        "mission_state": "completed",
        "total_specs": 2,
        "specs_run": 2,
        "succeeded_count": 2,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_batch_failure": False,
        "failed_spec_indices": [],
        "succeeded_spec_indices": [0, 1],
        "first_failed_spec_index": None,
        "last_spec_index_run": 1,
        "message": "x",
    }
    v = validate_team_named_mission_specs_summary(s)
    assert v["ok"] is True
    assert v["overall_outcome"] == "success"


def test_validate_team_named_mission_specs_summary_rejects_success_with_failures() -> None:
    s = {
        "ok": True,
        "overall_outcome": "success",
        "mission_state": "completed",
        "total_specs": 2,
        "specs_run": 2,
        "succeeded_count": 1,
        "failed_count": 1,
        "stopped_early": False,
        "continue_on_batch_failure": True,
        "failed_spec_indices": [1],
        "succeeded_spec_indices": [0],
        "first_failed_spec_index": 1,
        "last_spec_index_run": 1,
        "message": "x",
    }
    v = validate_team_named_mission_specs_summary(s)
    assert v["ok"] is False
    assert v["errors"]


def test_execute_team_named_mission_specs_malformed_input() -> None:
    run = {
        "ok": False,
        "overall_outcome": "error",
        "total_specs": 0,
        "specs_run": 0,
        "succeeded_count": 0,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_batch_failure": False,
        "results": [],
        "summary": {"message": "", "error": "specs must be a list"},
    }
    ex = {
        "ok": False,
        "overall_outcome": "error",
        "mission_state": "invalid",
        "total_specs": 0,
        "specs_run": 0,
        "succeeded_count": 0,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_batch_failure": False,
        "failed_spec_indices": [],
        "succeeded_spec_indices": [],
        "first_failed_spec_index": None,
        "last_spec_index_run": None,
        "message": "Invalid batch execution result.",
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_specs",
        return_value=run,
    ), patch(
        "multi_robot_mission_stack.coordinator.coordinator.summarize_team_named_mission_specs_result",
        return_value=ex,
    ):
        out = execute_team_named_mission_specs("bad")  # type: ignore[arg-type]

    assert out["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["summary"]["message"] == "Invalid batch mission execution."


def test_execute_team_named_mission_spec_malformed_summary() -> None:
    run = {"ok": True, "mode": "sequence", "overall_outcome": "success", "summary": {}}
    ex = {
        "ok": False,
        "mode": "invalid",
        "overall_outcome": "error",
        "mission_state": "invalid",
        "total_steps": 0,
        "steps_run": 0,
        "succeeded_count": 0,
        "failed_count": 0,
        "stopped_early": None,
        "continue_on_failure": None,
        "failed_step_indices": [],
        "succeeded_step_indices": [],
        "first_failed_step_index": None,
        "last_step_index_run": None,
        "message": "Invalid team mission result.",
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_spec",
        return_value=run,
    ), patch(
        "multi_robot_mission_stack.coordinator.coordinator.summarize_team_named_mission_result",
        return_value=ex,
    ):
        out = execute_team_named_mission_spec({})

    assert out["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["summary"]["error"] == "Invalid team mission result."


def test_execute_team_named_mission_spec_checked_sequence_success() -> None:
    run = {
        "ok": True,
        "mode": "sequence",
        "overall_outcome": "success",
        "summary": {"overall_outcome": "success", "total_steps": 1, "steps_run": 1, "steps": []},
    }
    ex = {
        "ok": True,
        "mode": "sequence",
        "overall_outcome": "success",
        "mission_state": "completed",
        "total_steps": 1,
        "steps_run": 1,
        "succeeded_count": 1,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_failure": False,
        "failed_step_indices": [],
        "succeeded_step_indices": [0],
        "first_failed_step_index": None,
        "last_step_index_run": 0,
        "message": "Sequence mission completed successfully.",
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_spec",
        return_value=run,
    ), patch(
        "multi_robot_mission_stack.coordinator.coordinator.summarize_team_named_mission_result",
        return_value=ex,
    ):
        out = execute_team_named_mission_spec_checked({})

    assert out["ok"] is True
    assert out["overall_outcome"] == "success"
    assert out["execution"]["ok"] is True
    assert out["validation"]["ok"] is True
    assert out["summary"]["message"] == "Mission spec executed and validated successfully."
    assert out["summary"]["error"] is None


def test_execute_team_named_mission_spec_checked_failed_fast() -> None:
    run = {
        "ok": False,
        "mode": "sequence",
        "overall_outcome": "failure",
        "summary": {},
    }
    ex = {
        "ok": False,
        "mode": "sequence",
        "overall_outcome": "failure",
        "mission_state": "failed_fast",
        "total_steps": 1,
        "steps_run": 1,
        "succeeded_count": 0,
        "failed_count": 1,
        "stopped_early": True,
        "continue_on_failure": False,
        "failed_step_indices": [0],
        "succeeded_step_indices": [],
        "first_failed_step_index": 0,
        "last_step_index_run": 0,
        "message": "Sequence mission failed at step 1 and stopped early.",
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_spec",
        return_value=run,
    ), patch(
        "multi_robot_mission_stack.coordinator.coordinator.summarize_team_named_mission_result",
        return_value=ex,
    ):
        out = execute_team_named_mission_spec_checked({})

    assert out["ok"] is False
    assert out["overall_outcome"] == "failure"
    assert out["execution"]["ok"] is False
    assert out["validation"]["ok"] is True
    assert out["summary"]["error"] == "Sequence mission failed at step 1 and stopped early."


def test_execute_team_named_mission_spec_checked_invalid_summary() -> None:
    fake_exe = {
        "ok": True,
        "mode": "sequence",
        "overall_outcome": "success",
        "run_result": {"ok": True, "mode": "sequence", "overall_outcome": "success", "summary": {}},
        "execution_summary": {"ok": True, "mode": "sequence"},
        "summary": {"message": "Mission spec executed successfully.", "error": None},
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.execute_team_named_mission_spec",
        return_value=fake_exe,
    ):
        out = execute_team_named_mission_spec_checked({})

    assert out["ok"] is False
    assert out["validation"]["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["summary"]["error"] == "Mission summary is invalid."


def test_execute_team_named_mission_spec_checked_malformed_execution() -> None:
    fake_exe = {
        "ok": False,
        "mode": "invalid",
        "overall_outcome": "error",
        "run_result": {},
        "summary": {"message": "", "error": "run failed"},
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.execute_team_named_mission_spec",
        return_value=fake_exe,
    ):
        out = execute_team_named_mission_spec_checked({})

    assert out["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["validation"]["ok"] is False


def test_execute_team_named_mission_specs_checked_two_success() -> None:
    run = {
        "ok": True,
        "overall_outcome": "success",
        "total_specs": 2,
        "specs_run": 2,
        "succeeded_count": 2,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_batch_failure": False,
        "results": [],
        "summary": {"message": "ok", "error": None},
    }
    ex = {
        "ok": True,
        "overall_outcome": "success",
        "mission_state": "completed",
        "total_specs": 2,
        "specs_run": 2,
        "succeeded_count": 2,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_batch_failure": False,
        "failed_spec_indices": [],
        "succeeded_spec_indices": [0, 1],
        "first_failed_spec_index": None,
        "last_spec_index_run": 1,
        "message": "Batch execution completed successfully.",
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_specs",
        return_value=run,
    ), patch(
        "multi_robot_mission_stack.coordinator.coordinator.summarize_team_named_mission_specs_result",
        return_value=ex,
    ):
        out = execute_team_named_mission_specs_checked([{}, {}])

    assert out["ok"] is True
    assert out["overall_outcome"] == "success"
    assert out["execution"]["ok"] is True
    assert out["validation"]["ok"] is True
    assert out["summary"]["message"] == "Batch mission specs executed and validated successfully."
    assert out["summary"]["error"] is None


def test_execute_team_named_mission_specs_checked_stop_early() -> None:
    run = {
        "ok": False,
        "overall_outcome": "failure",
        "total_specs": 2,
        "specs_run": 1,
        "succeeded_count": 0,
        "failed_count": 1,
        "stopped_early": True,
        "continue_on_batch_failure": False,
        "results": [{"index": 0, "ok": False, "mode": "sequence", "overall_outcome": "failure", "summary": {}}],
        "summary": {"message": "stopped", "error": None},
    }
    ex = {
        "ok": False,
        "overall_outcome": "failure",
        "mission_state": "failed_fast",
        "total_specs": 2,
        "specs_run": 1,
        "succeeded_count": 0,
        "failed_count": 1,
        "stopped_early": True,
        "continue_on_batch_failure": False,
        "failed_spec_indices": [0],
        "succeeded_spec_indices": [],
        "first_failed_spec_index": 0,
        "last_spec_index_run": 0,
        "message": "Batch execution failed at spec 1 and stopped early.",
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_specs",
        return_value=run,
    ), patch(
        "multi_robot_mission_stack.coordinator.coordinator.summarize_team_named_mission_specs_result",
        return_value=ex,
    ):
        out = execute_team_named_mission_specs_checked([{}, {}])

    assert out["ok"] is False
    assert out["execution"]["ok"] is False
    assert out["execution"]["execution_summary"]["mission_state"] == "failed_fast"
    assert out["validation"]["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["summary"]["error"] == "Batch mission summary is invalid."


def test_execute_team_named_mission_specs_checked_invalid_summary() -> None:
    fake_exe = {
        "ok": True,
        "overall_outcome": "success",
        "run_result": {"ok": True, "overall_outcome": "success", "total_specs": 1, "specs_run": 1, "summary": {}},
        "execution_summary": {"ok": True, "overall_outcome": "success"},
        "summary": {"message": "Batch mission specs executed successfully.", "error": None},
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.execute_team_named_mission_specs",
        return_value=fake_exe,
    ):
        out = execute_team_named_mission_specs_checked([{}])

    assert out["ok"] is False
    assert out["validation"]["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["summary"]["error"] == "Batch mission summary is invalid."


def test_execute_team_named_mission_specs_checked_malformed_execution() -> None:
    fake_exe = {
        "ok": False,
        "overall_outcome": "error",
        "run_result": {},
        "summary": {"message": "", "error": "bad"},
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.execute_team_named_mission_specs",
        return_value=fake_exe,
    ):
        out = execute_team_named_mission_specs_checked([])

    assert out["ok"] is False
    assert out["overall_outcome"] == "error"
    assert out["validation"]["ok"] is False


def test_inspect_team_named_mission_specs_order_preserved() -> None:
    order: list[int] = []

    def _fake(spec: Any) -> Dict[str, Any]:
        order.append(id(spec))
        return {
            "ok": True,
            "mode": "sequence",
            "overall_outcome": "success",
            "step_count": 0,
            "preflight": {"ok": True, "overall_outcome": "success", "summary": {"message": "", "error": None}},
            "preview": {
                "ok": True,
                "overall_outcome": "success",
                "lines": [],
                "summary": {"message": "", "error": None},
            },
            "summary": {"message": "", "error": None},
        }

    s1 = {"x": 1}
    s2 = {"x": 2}
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.inspect_team_named_mission_spec",
        side_effect=_fake,
    ):
        out = inspect_team_named_mission_specs([s1, s2])

    assert order == [id(s1), id(s2)]
    assert [r["index"] for r in out["results"]] == [0, 1]


def test_validate_team_named_mission_spec_contract_valid() -> None:
    spec = {
        "version": "v1",
        "mode": "sequence",
        "steps": [{"robot_id": "r1", "location_name": "a"}],
    }
    v = validate_team_named_mission_spec_contract(spec)
    assert v["ok"] is True
    assert v["overall_outcome"] == "success"
    assert v["errors"] == []


def test_validate_team_named_mission_spec_contract_missing_version() -> None:
    spec = {"mode": "sequence", "steps": [{"robot_id": "r1", "location_name": "a"}]}
    v = validate_team_named_mission_spec_contract(spec)
    assert v["ok"] is False
    assert any("version" in e.lower() for e in v["errors"])


def test_validate_team_named_mission_spec_contract_parallel_duplicate_robot() -> None:
    spec = {
        "version": "v1",
        "mode": "parallel",
        "steps": [
            {"robot_id": "r1", "location_name": "a"},
            {"robot_id": "r1", "location_name": "b"},
        ],
    }
    v = validate_team_named_mission_spec_contract(spec)
    assert v["ok"] is False
    assert any("duplicate" in e.lower() for e in v["errors"])


def test_validate_team_named_mission_specs_contract_valid() -> None:
    specs = [
        {"version": "v1", "mode": "sequence", "steps": [{"robot_id": "a", "location_name": "x"}]},
        {
            "version": "v1",
            "mode": "parallel",
            "steps": [
                {"robot_id": "r1", "location_name": "a"},
                {"robot_id": "r2", "location_name": "b"},
            ],
        },
    ]
    v = validate_team_named_mission_specs_contract(specs)
    assert v["ok"] is True
    assert v["spec_count"] == 2


def test_validate_team_named_mission_execution_contract_valid_execute_shape() -> None:
    payload = {
        "ok": True,
        "version": "v1",
        "mode": "sequence",
        "overall_outcome": "success",
        "run_result": {"ok": True, "mode": "sequence", "overall_outcome": "success", "summary": {}},
        "execution_summary": {
            "ok": True,
            "version": "v1",
            "mode": "sequence",
            "overall_outcome": "success",
            "mission_state": "completed",
            "total_steps": 1,
            "steps_run": 1,
            "succeeded_count": 1,
            "failed_count": 0,
            "stopped_early": False,
            "continue_on_failure": False,
            "failed_step_indices": [],
            "succeeded_step_indices": [0],
            "first_failed_step_index": None,
            "last_step_index_run": 0,
            "message": "x",
        },
        "summary": {"message": "", "error": None},
    }
    v = validate_team_named_mission_execution_contract(payload)
    assert v["ok"] is True


def test_validate_team_named_mission_execution_contract_malformed() -> None:
    v = validate_team_named_mission_execution_contract({"ok": True, "version": "v1"})
    assert v["ok"] is False
    assert v["errors"]


def test_validate_team_named_mission_specs_execution_contract_valid() -> None:
    payload = {
        "ok": True,
        "version": "v1",
        "overall_outcome": "success",
        "run_result": {
            "ok": True,
            "overall_outcome": "success",
            "total_specs": 1,
            "specs_run": 1,
            "succeeded_count": 1,
            "failed_count": 0,
            "stopped_early": False,
            "continue_on_batch_failure": False,
            "results": [],
            "summary": {},
        },
        "execution_summary": {
            "ok": True,
            "version": "v1",
            "overall_outcome": "success",
            "mission_state": "completed",
            "total_specs": 1,
            "specs_run": 1,
            "succeeded_count": 1,
            "failed_count": 0,
            "stopped_early": False,
            "continue_on_batch_failure": False,
            "failed_spec_indices": [],
            "succeeded_spec_indices": [0],
            "first_failed_spec_index": None,
            "last_spec_index_run": 0,
            "message": "x",
        },
        "summary": {"message": "", "error": None},
    }
    v = validate_team_named_mission_specs_execution_contract(payload)
    assert v["ok"] is True


def test_validate_team_named_mission_specs_execution_contract_malformed() -> None:
    v = validate_team_named_mission_specs_execution_contract({"version": "v1"})
    assert v["ok"] is False
    assert v["errors"]


def test_stable_surfaces_include_contract_version_v1_inspect() -> None:
    spec = {"mode": "sequence", "steps": [{"robot_id": "r1", "location_name": "a"}]}
    out = inspect_team_named_mission_spec(spec)
    assert out.get("version") == "v1"


def test_stable_surfaces_include_contract_version_v1_execute_mocked() -> None:
    run = {
        "ok": True,
        "mode": "sequence",
        "overall_outcome": "success",
        "summary": {
            "overall_outcome": "success",
            "total_steps": 1,
            "steps_run": 1,
            "succeeded_count": 1,
            "failed_count": 0,
            "stopped_early": False,
            "continue_on_failure": False,
            "steps": [{"index": 0, "step_outcome": "succeeded"}],
        },
    }
    ex = {
        "ok": True,
        "version": "v1",
        "mode": "sequence",
        "overall_outcome": "success",
        "mission_state": "completed",
        "total_steps": 1,
        "steps_run": 1,
        "succeeded_count": 1,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_failure": False,
        "failed_step_indices": [],
        "succeeded_step_indices": [0],
        "first_failed_step_index": None,
        "last_step_index_run": 0,
        "message": "Sequence mission completed successfully.",
    }
    with patch(
        "multi_robot_mission_stack.coordinator.coordinator.run_team_named_mission_spec",
        return_value=run,
    ), patch(
        "multi_robot_mission_stack.coordinator.coordinator.summarize_team_named_mission_result",
        return_value=ex,
    ):
        out = execute_team_named_mission_spec({})
    assert out.get("version") == "v1"


def test_stable_surfaces_include_contract_version_v1_summary() -> None:
    result = {
        "ok": True,
        "mode": "sequence",
        "overall_outcome": "success",
        "summary": {
            "overall_outcome": "success",
            "total_steps": 1,
            "steps_run": 1,
            "succeeded_count": 1,
            "failed_count": 0,
            "stopped_early": False,
            "continue_on_failure": False,
            "steps": [{"index": 0, "step_outcome": "succeeded"}],
        },
    }
    s = summarize_team_named_mission_result(result)
    assert s.get("version") == "v1"
