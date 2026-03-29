"""Layer B: minimal team-facing coordinator over the validated execution stack."""

from __future__ import annotations

import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List, Tuple

from ..agent.mission_agent_facade import MissionAgentFacade
from ..agent.sequence_utils import _navigate_named_command, _navigate_ok, _validate_steps
from ..agent.wait_utils import wait_for_terminal_navigation_state


def assign_named_navigation(
    robot_id: str,
    location_name: str,
    *,
    per_goal_timeout_sec: float = 120.0,
    poll_interval_sec: float = 1.0,
    bridge_node_name: str = "mission_bridge_node",
) -> Dict[str, Any]:
    """
    One robot, one named-navigation leg: navigate via facade, then wait for terminal state.

    Constructs and closes a ``MissionAgentFacade`` per call (no shared client, no concurrency).
    """
    rid = str(robot_id).strip()
    loc = str(location_name).strip()
    start = time.monotonic()
    facade: MissionAgentFacade | None = None

    def _elapsed() -> float:
        return round(time.monotonic() - start, 3)

    try:
        facade = MissionAgentFacade.with_ros(bridge_node_name=bridge_node_name)
    except Exception as exc:
        return {
            "robot_id": rid,
            "location_name": loc,
            "goal_id": "",
            "outcome": "failed",
            "status": "failed",
            "nav_status": "unknown",
            "message": str(exc),
            "elapsed_sec": _elapsed(),
        }

    try:
        try:
            nav = facade.handle_command(_navigate_named_command(rid, loc))
        except Exception as exc:
            return {
                "robot_id": rid,
                "location_name": loc,
                "goal_id": "",
                "outcome": "failed",
                "status": "failed",
                "nav_status": "unknown",
                "message": str(exc),
                "elapsed_sec": _elapsed(),
            }

        if not isinstance(nav, dict):
            return {
                "robot_id": rid,
                "location_name": loc,
                "goal_id": "",
                "outcome": "failed",
                "status": "failed",
                "nav_status": "unknown",
                "message": "invalid_navigate_result",
                "elapsed_sec": _elapsed(),
            }

        ok, reason = _navigate_ok(nav)
        if not ok:
            return {
                "robot_id": rid,
                "location_name": loc,
                "goal_id": "",
                "outcome": "failed",
                "status": str(nav.get("status", "")),
                "nav_status": str(nav.get("nav_status", "")),
                "message": str(nav.get("message", "") or reason),
                "elapsed_sec": _elapsed(),
            }

        gid = str(nav["goal_id"]).strip()
        try:
            wait_res = wait_for_terminal_navigation_state(
                facade,
                rid,
                gid,
                timeout_sec=float(per_goal_timeout_sec),
                poll_interval_sec=float(poll_interval_sec),
            )
        except Exception as exc:
            return {
                "robot_id": rid,
                "location_name": loc,
                "goal_id": gid,
                "outcome": "failed",
                "status": "failed",
                "nav_status": "unknown",
                "message": str(exc),
                "elapsed_sec": _elapsed(),
            }

        woc = str(wait_res.get("outcome", "")).lower() or "unknown"
        return {
            "robot_id": rid,
            "location_name": loc,
            "goal_id": gid,
            "outcome": woc,
            "status": str(wait_res.get("status", "")),
            "nav_status": str(wait_res.get("nav_status", "")),
            "message": str(wait_res.get("message", "")),
            "elapsed_sec": _elapsed(),
        }
    finally:
        if facade is not None:
            try:
                facade.close()
            except Exception:
                pass


def assign_named_sequence(
    steps: List[Dict[str, str]],
    *,
    per_goal_timeout_sec: float = 120.0,
    poll_interval_sec: float = 1.0,
    continue_on_failure: bool = False,
    bridge_node_name: str = "mission_bridge_node",
) -> Dict[str, Any]:
    """
    Sequential team mission: each step is one ``assign_named_navigation`` leg (own facade per leg).
    """
    err = _validate_steps(steps)
    if err is not None:
        n = len(steps) if isinstance(steps, list) else 0
        return {
            "overall_outcome": "failure",
            "total_steps": n,
            "steps_run": 0,
            "succeeded_count": 0,
            "failed_count": 0,
            "stopped_early": False,
            "continue_on_failure": bool(continue_on_failure),
            "steps": [],
            "error": err,
        }

    total = len(steps)
    out_steps: List[Dict[str, Any]] = []
    succeeded_count = 0
    failed_count = 0
    cont = bool(continue_on_failure)

    for idx, step in enumerate(steps):
        rid = str(step["robot_id"]).strip()
        loc = str(step["location_name"]).strip()
        leg = assign_named_navigation(
            rid,
            loc,
            per_goal_timeout_sec=float(per_goal_timeout_sec),
            poll_interval_sec=float(poll_interval_sec),
            bridge_node_name=bridge_node_name,
        )
        oc = str(leg.get("outcome", "")).lower()
        step_ok = oc == "succeeded"
        if step_ok:
            succeeded_count += 1
            step_outcome = "succeeded"
        else:
            failed_count += 1
            step_outcome = "failed"

        out_steps.append(
            {
                "index": idx,
                "robot_id": rid,
                "location_name": loc,
                "result": leg,
                "step_outcome": step_outcome,
            }
        )

        if not step_ok and not cont:
            break

    steps_run = len(out_steps)
    stopped_early = steps_run < total and not cont and failed_count > 0
    overall = (
        "success"
        if failed_count == 0 and steps_run == total
        else "failure"
    )

    return {
        "overall_outcome": overall,
        "total_steps": total,
        "steps_run": steps_run,
        "succeeded_count": succeeded_count,
        "failed_count": failed_count,
        "stopped_early": stopped_early,
        "continue_on_failure": cont,
        "steps": out_steps,
    }


def assign_named_parallel(
    steps: List[Dict[str, str]],
    *,
    per_goal_timeout_sec: float = 120.0,
    poll_interval_sec: float = 1.0,
    max_workers: int | None = None,
    bridge_node_name: str = "mission_bridge_node",
) -> Dict[str, Any]:
    """
    Parallel team batch: one ``assign_named_navigation`` per step (own facade per leg; threads only).

    Rejects duplicate ``robot_id`` values in the same batch (no concurrent legs for one robot).
    """

    def _error(
        total_steps: int,
        pool_workers: int,
        message: str,
    ) -> Dict[str, Any]:
        return {
            "overall_outcome": "error",
            "total_steps": int(total_steps),
            "steps_run": 0,
            "succeeded_count": 0,
            "failed_count": 0,
            "max_workers": int(pool_workers),
            "steps": [],
            "error": message,
        }

    err = _validate_steps(steps)
    if err is not None:
        n = len(steps) if isinstance(steps, list) else 0
        try:
            pw = max(1, int(max_workers)) if max_workers is not None else max(1, n or 1)
        except (TypeError, ValueError):
            pw = max(1, n or 1)
        return _error(n, pw, err)

    if max_workers is not None:
        try:
            mw = int(max_workers)
        except (TypeError, ValueError):
            return _error(len(steps), max(1, len(steps) or 1), "max_workers must be a positive int")
        if mw < 1:
            return _error(len(steps), 1, "max_workers must be a positive int")
        pool_workers = mw
    else:
        pool_workers = max(1, len(steps) or 1)

    total = len(steps)
    seen: set[str] = set()
    for step in steps:
        rid = str(step["robot_id"]).strip()
        if rid in seen:
            return _error(
                total,
                pool_workers,
                "duplicate robot_id in parallel batch",
            )
        seen.add(rid)

    def _leg(item: Tuple[int, Dict[str, str]]) -> Tuple[int, str, str, Dict[str, Any]]:
        idx, step = item
        r = str(step["robot_id"]).strip()
        loc = str(step["location_name"]).strip()
        leg = assign_named_navigation(
            r,
            loc,
            per_goal_timeout_sec=float(per_goal_timeout_sec),
            poll_interval_sec=float(poll_interval_sec),
            bridge_node_name=bridge_node_name,
        )
        return idx, r, loc, leg

    out_steps: List[Dict[str, Any]] = []
    succeeded_count = 0
    failed_count = 0

    with ThreadPoolExecutor(max_workers=pool_workers) as ex:
        for idx, rid, loc, leg in ex.map(_leg, [(i, s) for i, s in enumerate(steps)]):
            oc = str(leg.get("outcome", "")).lower()
            step_ok = oc == "succeeded"
            if step_ok:
                succeeded_count += 1
                step_outcome = "succeeded"
            else:
                failed_count += 1
                step_outcome = "failed"
            out_steps.append(
                {
                    "index": idx,
                    "robot_id": rid,
                    "location_name": loc,
                    "result": leg,
                    "step_outcome": step_outcome,
                }
            )

    steps_run = len(out_steps)
    overall = "success" if failed_count == 0 and steps_run == total else "failure"

    return {
        "overall_outcome": overall,
        "total_steps": total,
        "steps_run": steps_run,
        "succeeded_count": succeeded_count,
        "failed_count": failed_count,
        "max_workers": pool_workers,
        "steps": out_steps,
    }


def normalize_team_named_mission_spec(
    steps: Any,
    mode: str = "sequence",
) -> Dict[str, Any]:
    """Pure normalization/validation for ``assign_team_named_mission`` inputs (no ROS, no dispatch)."""
    m = str(mode).strip().lower()
    if m not in ("sequence", "parallel"):
        return {
            "ok": False,
            "mode": m,
            "steps": [],
            "error": "unsupported mode",
        }
    if not isinstance(steps, list):
        return {
            "ok": False,
            "mode": m,
            "steps": [],
            "error": "steps must be a list",
        }
    norm_steps: List[Dict[str, str]] = []
    for i, raw in enumerate(steps):
        if not isinstance(raw, dict):
            return {
                "ok": False,
                "mode": m,
                "steps": [],
                "error": f"step {i} must be a dict",
            }
        rid = raw.get("robot_id")
        loc = raw.get("location_name")
        rs = "" if rid is None else str(rid).strip()
        ls = "" if loc is None else str(loc).strip()
        if not rs:
            return {
                "ok": False,
                "mode": m,
                "steps": [],
                "error": f"step {i} has empty robot_id",
            }
        if not ls:
            return {
                "ok": False,
                "mode": m,
                "steps": [],
                "error": f"step {i} has empty location_name",
            }
        norm_steps.append({"robot_id": rs, "location_name": ls})
    return {
        "ok": True,
        "mode": m,
        "steps": norm_steps,
        "error": None,
    }


def assign_team_named_mission(
    steps: Any,
    mode: str = "sequence",
    *,
    per_goal_timeout_sec: float = 120.0,
    poll_interval_sec: float = 1.0,
    continue_on_failure: bool = False,
    max_workers: int | None = None,
    bridge_node_name: str = "mission_bridge_node",
) -> Dict[str, Any]:
    """Dispatch team named navigation to sequence or parallel coordinator helpers."""
    norm = normalize_team_named_mission_spec(steps, mode)
    if not norm["ok"]:
        return {
            "mode": norm["mode"],
            "ok": False,
            "overall_outcome": "error",
            "summary": {"error": str(norm["error"])},
        }

    m = str(norm["mode"])
    norm_steps: List[Dict[str, str]] = norm["steps"]
    if m == "sequence":
        summary = assign_named_sequence(
            norm_steps,
            per_goal_timeout_sec=float(per_goal_timeout_sec),
            poll_interval_sec=float(poll_interval_sec),
            continue_on_failure=bool(continue_on_failure),
            bridge_node_name=bridge_node_name,
        )
        label = "sequence"
    else:
        summary = assign_named_parallel(
            norm_steps,
            per_goal_timeout_sec=float(per_goal_timeout_sec),
            poll_interval_sec=float(poll_interval_sec),
            max_workers=max_workers,
            bridge_node_name=bridge_node_name,
        )
        label = "parallel"

    oo = summary["overall_outcome"] if "overall_outcome" in summary else "error"
    return {
        "mode": label,
        "ok": oo == "success",
        "overall_outcome": oo,
        "summary": summary,
    }


def run_team_named_mission_spec(spec: Any) -> Dict[str, Any]:
    """Run a team mission from a single spec dict (wrapper over ``assign_team_named_mission``)."""
    if not isinstance(spec, dict):
        return {
            "ok": False,
            "mode": "invalid",
            "overall_outcome": "error",
            "summary": {"error": "spec must be a dict"},
        }
    raw_opts = spec.get("options", {})
    if not isinstance(raw_opts, dict):
        return {
            "ok": False,
            "mode": "invalid",
            "overall_outcome": "error",
            "summary": {"error": "options must be a dict"},
        }
    keys = (
        "per_goal_timeout_sec",
        "poll_interval_sec",
        "continue_on_failure",
        "max_workers",
        "bridge_node_name",
    )
    opts = {k: raw_opts[k] for k in keys if k in raw_opts}
    return assign_team_named_mission(
        spec.get("steps", []),
        spec.get("mode", "sequence"),
        per_goal_timeout_sec=float(opts.get("per_goal_timeout_sec", 120.0)),
        poll_interval_sec=float(opts.get("poll_interval_sec", 1.0)),
        continue_on_failure=bool(opts.get("continue_on_failure", False)),
        max_workers=opts.get("max_workers"),
        bridge_node_name=str(opts.get("bridge_node_name", "mission_bridge_node")),
    )


def run_team_named_mission_specs(
    specs: Any,
    continue_on_batch_failure: bool = False,
) -> Dict[str, Any]:
    """Run several mission specs sequentially at batch level (each spec uses ``run_team_named_mission_spec``)."""
    cont = bool(continue_on_batch_failure)
    if not isinstance(specs, list):
        return {
            "ok": False,
            "overall_outcome": "error",
            "total_specs": 0,
            "specs_run": 0,
            "succeeded_count": 0,
            "failed_count": 0,
            "stopped_early": False,
            "continue_on_batch_failure": cont,
            "results": [],
            "summary": {"message": "", "error": "specs must be a list"},
        }

    total = len(specs)
    results: List[Dict[str, Any]] = []
    succeeded_count = 0
    failed_count = 0

    for i, spec in enumerate(specs):
        res = run_team_named_mission_spec(spec)
        ok = bool(res.get("ok"))
        if ok:
            succeeded_count += 1
        else:
            failed_count += 1
        inner = res.get("summary") if isinstance(res.get("summary"), dict) else {}
        results.append(
            {
                "index": i,
                "ok": ok,
                "mode": str(res.get("mode", "invalid")),
                "overall_outcome": str(res.get("overall_outcome", "error")),
                "summary": inner,
            }
        )
        if not ok and not cont:
            break

    specs_run = len(results)
    stopped_early = specs_run < total and not cont and failed_count > 0

    if failed_count == 0:
        batch_ok = True
        overall = "success"
        if total == 1:
            msg = "Executed 1 mission spec successfully."
        else:
            msg = f"Executed {total} mission specs successfully."
        top_err = None
    else:
        batch_ok = False
        overall = "failure"
        if stopped_early:
            msg = "Execution stopped after 1 failed mission spec."
        elif failed_count == 1:
            msg = "Execution completed with 1 failed mission spec."
        else:
            msg = f"Execution completed with {failed_count} failed mission specs."
        top_err = None

    return {
        "ok": batch_ok,
        "overall_outcome": overall,
        "total_specs": total,
        "specs_run": specs_run,
        "succeeded_count": succeeded_count,
        "failed_count": failed_count,
        "stopped_early": stopped_early,
        "continue_on_batch_failure": cont,
        "results": results,
        "summary": {"message": msg, "error": top_err},
    }


def summarize_team_named_mission_specs_result(batch_result: Any) -> Dict[str, Any]:
    """Compact interpretation of ``run_team_named_mission_specs`` output (pure, no ROS)."""

    def _invalid() -> Dict[str, Any]:
        return {
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

    if not isinstance(batch_result, dict):
        return _invalid()

    oo_raw = batch_result.get("overall_outcome")
    if oo_raw not in ("success", "failure"):
        return _invalid()

    rows = batch_result.get("results")
    if not isinstance(rows, list):
        return _invalid()

    try:
        total_specs = int(batch_result["total_specs"])
        specs_run = int(batch_result["specs_run"])
        succeeded_count = int(batch_result["succeeded_count"])
        failed_count = int(batch_result["failed_count"])
    except (KeyError, TypeError, ValueError):
        return _invalid()

    stopped_early = bool(batch_result.get("stopped_early", False))
    cont = bool(batch_result.get("continue_on_batch_failure", False))

    failed_spec_indices: List[int] = []
    succeeded_spec_indices: List[int] = []
    for row in rows:
        if not isinstance(row, dict) or "index" not in row or "ok" not in row:
            return _invalid()
        try:
            idx = int(row["index"])
        except (TypeError, ValueError):
            return _invalid()
        if row["ok"] is True:
            succeeded_spec_indices.append(idx)
        else:
            failed_spec_indices.append(idx)

    failed_spec_indices.sort()
    succeeded_spec_indices.sort()

    if failed_spec_indices or succeeded_spec_indices:
        last_spec_index_run = max(failed_spec_indices + succeeded_spec_indices)
    else:
        last_spec_index_run = None

    first_failed_spec_index = failed_spec_indices[0] if failed_spec_indices else None

    if oo_raw == "success":
        overall_outcome = "success"
        mission_state = "completed"
        message = "Batch execution completed successfully."
    else:
        overall_outcome = "failure"
        if stopped_early and failed_spec_indices:
            mission_state = "failed_fast"
            n = (first_failed_spec_index + 1) if first_failed_spec_index is not None else 0
            message = f"Batch execution failed at spec {n} and stopped early."
        else:
            mission_state = "partial_failure"
            fc = len(failed_spec_indices)
            if fc == 1:
                message = "Batch execution completed with 1 failed spec."
            else:
                message = f"Batch execution completed with {fc} failed specs."

    return {
        "ok": overall_outcome == "success",
        "overall_outcome": overall_outcome,
        "mission_state": mission_state,
        "total_specs": total_specs,
        "specs_run": specs_run,
        "succeeded_count": succeeded_count,
        "failed_count": failed_count,
        "stopped_early": stopped_early,
        "continue_on_batch_failure": cont,
        "failed_spec_indices": list(failed_spec_indices),
        "succeeded_spec_indices": list(succeeded_spec_indices),
        "first_failed_spec_index": first_failed_spec_index,
        "last_spec_index_run": last_spec_index_run,
        "message": message,
    }


def execute_team_named_mission_specs(
    specs: Any,
    continue_on_batch_failure: bool = False,
) -> Dict[str, Any]:
    """Run several mission specs and attach a compact batch execution summary (ROS via batch runner)."""
    run = run_team_named_mission_specs(specs, continue_on_batch_failure=continue_on_batch_failure)
    ex = summarize_team_named_mission_specs_result(run)

    run_ok = bool(run.get("ok"))
    ex_ok = bool(ex.get("ok"))
    top_ok = run_ok and ex_ok

    run_compact = {
        "ok": run_ok,
        "overall_outcome": str(run.get("overall_outcome", "error")),
        "total_specs": int(run.get("total_specs", 0)),
        "specs_run": int(run.get("specs_run", 0)),
        "succeeded_count": int(run.get("succeeded_count", 0)),
        "failed_count": int(run.get("failed_count", 0)),
        "stopped_early": bool(run.get("stopped_early", False)),
        "continue_on_batch_failure": bool(run.get("continue_on_batch_failure", False)),
        "results": list(run.get("results", [])),
        "summary": run.get("summary") if isinstance(run.get("summary"), dict) else {},
    }

    if top_ok:
        top_oo = "success"
        top_msg = "Batch mission specs executed successfully."
        top_err = None
    elif ex.get("overall_outcome") == "error" or ex.get("mission_state") == "invalid":
        top_oo = str(ex.get("overall_outcome", "error"))
        top_msg = "Invalid batch mission execution."
        top_err = None
    else:
        top_oo = str(ex.get("overall_outcome", run.get("overall_outcome", "error")))
        top_msg = str(ex.get("message", ""))
        top_err = None

    return {
        "ok": top_ok,
        "overall_outcome": top_oo,
        "run_result": run_compact,
        "execution_summary": ex,
        "summary": {"message": top_msg, "error": top_err},
    }


def validate_team_named_mission_summary(summary: Any) -> Dict[str, Any]:
    """Check internal consistency of ``summarize_team_named_mission_result`` output (pure, no ROS)."""
    bad = "Mission summary is invalid."
    good = "Mission summary is internally consistent."
    if not isinstance(summary, dict):
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": ["summary must be a dict"]}

    req = (
        "ok",
        "mode",
        "overall_outcome",
        "mission_state",
        "total_steps",
        "steps_run",
        "succeeded_count",
        "failed_count",
        "stopped_early",
        "continue_on_failure",
        "failed_step_indices",
        "succeeded_step_indices",
        "first_failed_step_index",
        "last_step_index_run",
    )
    for k in req:
        if k not in summary:
            return {"ok": False, "overall_outcome": "error", "message": bad, "errors": [f"missing {k}"]}

    try:
        total_steps = int(summary["total_steps"])
        int(summary["steps_run"])
        sc = int(summary["succeeded_count"])
        fc = int(summary["failed_count"])
    except (TypeError, ValueError):
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": ["invalid integer count fields"]}

    fi = summary["failed_step_indices"]
    si = summary["succeeded_step_indices"]
    if not isinstance(fi, list) or not isinstance(si, list):
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": ["step index fields must be lists"]}

    errors: List[str] = []
    oo = summary["overall_outcome"]
    if oo == "success":
        if fc != 0:
            errors.append("success requires failed_count==0")
        if len(fi) != 0:
            errors.append("success requires empty failed_step_indices")

    if sc + fc != total_steps:
        errors.append("succeeded_count + failed_count must equal total_steps")
    if len(si) != sc:
        errors.append("len(succeeded_step_indices) must equal succeeded_count")
    if len(fi) != fc:
        errors.append("len(failed_step_indices) must equal failed_count")

    if fc == 0:
        if summary["first_failed_step_index"] is not None:
            errors.append("first_failed_step_index must be null when failed_count==0")
    elif summary["first_failed_step_index"] is None:
        errors.append("first_failed_step_index must be set when failed_count>0")

    lir = summary["last_step_index_run"]
    if lir is not None:
        try:
            if int(lir) < 0:
                errors.append("last_step_index_run must be >= 0 when set")
        except (TypeError, ValueError):
            errors.append("last_step_index_run must be int-coercible or null")

    mode = str(summary["mode"]).strip().lower()
    ms = str(summary.get("mission_state", ""))
    if mode == "sequence":
        if ms == "failed_fast" and summary.get("stopped_early") is not True:
            errors.append("failed_fast requires stopped_early true")
    elif mode == "parallel":
        if summary.get("stopped_early") is not None:
            errors.append("parallel requires stopped_early null")
        if summary.get("continue_on_failure") is not None:
            errors.append("parallel requires continue_on_failure null")

    if errors:
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": errors}
    return {"ok": True, "overall_outcome": "success", "message": good, "errors": []}


def validate_team_named_mission_specs_summary(summary: Any) -> Dict[str, Any]:
    """Check internal consistency of ``summarize_team_named_mission_specs_result`` output (pure, no ROS)."""
    bad = "Batch mission summary is invalid."
    good = "Batch mission summary is internally consistent."
    if not isinstance(summary, dict):
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": ["summary must be a dict"]}

    req = (
        "ok",
        "overall_outcome",
        "mission_state",
        "total_specs",
        "specs_run",
        "succeeded_count",
        "failed_count",
        "stopped_early",
        "continue_on_batch_failure",
        "failed_spec_indices",
        "succeeded_spec_indices",
        "first_failed_spec_index",
        "last_spec_index_run",
    )
    for k in req:
        if k not in summary:
            return {"ok": False, "overall_outcome": "error", "message": bad, "errors": [f"missing {k}"]}

    try:
        total_specs = int(summary["total_specs"])
        int(summary["specs_run"])
        sc = int(summary["succeeded_count"])
        fc = int(summary["failed_count"])
    except (TypeError, ValueError):
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": ["invalid integer count fields"]}

    fi = summary["failed_spec_indices"]
    si = summary["succeeded_spec_indices"]
    if not isinstance(fi, list) or not isinstance(si, list):
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": ["spec index fields must be lists"]}

    errors: List[str] = []
    if summary["overall_outcome"] == "success":
        if fc != 0:
            errors.append("success requires failed_count==0")
        if len(fi) != 0:
            errors.append("success requires empty failed_spec_indices")

    if sc + fc != total_specs:
        errors.append("succeeded_count + failed_count must equal total_specs")
    if len(si) != sc:
        errors.append("len(succeeded_spec_indices) must equal succeeded_count")
    if len(fi) != fc:
        errors.append("len(failed_spec_indices) must equal failed_count")

    if fc == 0:
        if summary["first_failed_spec_index"] is not None:
            errors.append("first_failed_spec_index must be null when failed_count==0")
    elif summary["first_failed_spec_index"] is None:
        errors.append("first_failed_spec_index must be set when failed_count>0")

    lsr = summary["last_spec_index_run"]
    if lsr is not None:
        try:
            if int(lsr) < 0:
                errors.append("last_spec_index_run must be >= 0 when set")
        except (TypeError, ValueError):
            errors.append("last_spec_index_run must be int-coercible or null")

    if str(summary.get("mission_state", "")) == "failed_fast" and summary.get("stopped_early") is not True:
        errors.append("failed_fast requires stopped_early true")

    if errors:
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": errors}
    return {"ok": True, "overall_outcome": "success", "message": good, "errors": []}


def preflight_team_named_mission_spec(spec: Any) -> Dict[str, Any]:
    """Dry-run validation/preview for a team mission spec (no ROS, no dispatch)."""

    def _opts_payload(raw: Dict[str, Any]) -> Dict[str, Any]:
        keys = (
            "per_goal_timeout_sec",
            "poll_interval_sec",
            "continue_on_failure",
            "max_workers",
            "bridge_node_name",
        )
        picked = {k: raw[k] for k in keys if k in raw}
        return {
            "per_goal_timeout_sec": picked.get("per_goal_timeout_sec", 120.0),
            "poll_interval_sec": picked.get("poll_interval_sec", 1.0),
            "continue_on_failure": picked.get("continue_on_failure", False),
            "max_workers": picked.get("max_workers"),
            "bridge_node_name": picked.get("bridge_node_name", "mission_bridge_node"),
        }

    def _err(
        mode: str,
        *,
        steps: List[Dict[str, str]],
        step_count: int,
        pru: bool | None,
        opt: Dict[str, Any],
        err: str,
        msg: str = "",
    ) -> Dict[str, Any]:
        return {
            "ok": False,
            "mode": mode,
            "overall_outcome": "error",
            "normalized_steps": steps,
            "step_count": step_count,
            "parallel_robot_ids_unique": pru,
            "options": opt,
            "summary": {"message": msg, "error": err},
        }

    if not isinstance(spec, dict):
        return _err("invalid", steps=[], step_count=0, pru=None, opt={}, err="spec must be a dict")

    if "options" in spec and not isinstance(spec["options"], dict):
        return _err("invalid", steps=[], step_count=0, pru=None, opt={}, err="options must be a dict")

    raw_opts = spec["options"] if "options" in spec else {}
    opts_out = _opts_payload(raw_opts)

    norm = normalize_team_named_mission_spec(spec.get("steps", []), spec.get("mode", "sequence"))
    if not norm["ok"]:
        return _err(
            str(norm["mode"]),
            steps=[],
            step_count=0,
            pru=None,
            opt=opts_out,
            err=str(norm["error"]),
        )

    m = str(norm["mode"])
    nsteps: List[Dict[str, str]] = list(norm["steps"])
    sc = len(nsteps)

    if m == "parallel":
        rids = [s["robot_id"] for s in nsteps]
        if len(set(rids)) != len(rids):
            return _err(
                m,
                steps=nsteps,
                step_count=sc,
                pru=False,
                opt=opts_out,
                err="duplicate robot_id in parallel batch",
            )
        return {
            "ok": True,
            "mode": m,
            "overall_outcome": "success",
            "normalized_steps": nsteps,
            "step_count": sc,
            "parallel_robot_ids_unique": True,
            "options": opts_out,
            "summary": {
                "message": "Preflight passed for parallel mission.",
                "error": None,
            },
        }

    return {
        "ok": True,
        "mode": m,
        "overall_outcome": "success",
        "normalized_steps": nsteps,
        "step_count": sc,
        "parallel_robot_ids_unique": None,
        "options": opts_out,
        "summary": {
            "message": "Preflight passed for sequence mission.",
            "error": None,
        },
    }


def preflight_team_named_mission_specs(specs: Any) -> Dict[str, Any]:
    """Dry-run preflight for several mission specs in order (no ROS, no dispatch)."""
    if not isinstance(specs, list):
        return {
            "ok": False,
            "overall_outcome": "error",
            "total_specs": 0,
            "ok_count": 0,
            "error_count": 0,
            "results": [],
            "summary": {"message": "", "error": "specs must be a list"},
        }

    results: List[Dict[str, Any]] = []
    ok_count = 0
    error_count = 0
    for i, spec in enumerate(specs):
        pf = preflight_team_named_mission_spec(spec)
        pok = bool(pf.get("ok"))
        if pok:
            ok_count += 1
        else:
            error_count += 1
        s = pf.get("summary") if isinstance(pf.get("summary"), dict) else {}
        err = s.get("error")
        results.append(
            {
                "index": i,
                "ok": pok,
                "mode": str(pf.get("mode", "invalid")),
                "overall_outcome": str(pf.get("overall_outcome", "error")),
                "step_count": int(pf["step_count"]) if "step_count" in pf else 0,
                "summary": {
                    "message": str(s.get("message", "")),
                    "error": None if err is None else str(err),
                },
            }
        )

    n = len(specs)
    all_ok = error_count == 0
    if all_ok:
        msg = (
            "Preflight passed for 1 mission spec."
            if n == 1
            else f"Preflight passed for {n} mission specs."
        )
        top_err = None
    else:
        msg = (
            "Preflight completed with 1 invalid mission spec."
            if error_count == 1
            else f"Preflight completed with {error_count} invalid mission specs."
        )
        top_err = None

    return {
        "ok": all_ok,
        "overall_outcome": "success" if all_ok else "error",
        "total_specs": n,
        "ok_count": ok_count,
        "error_count": error_count,
        "results": results,
        "summary": {"message": msg, "error": top_err},
    }


def preview_team_named_mission_spec(spec: Any) -> Dict[str, Any]:
    """Human-readable preview lines for a team mission spec (uses ``preflight_team_named_mission_spec`` only)."""
    pf = preflight_team_named_mission_spec(spec)
    mode = str(pf.get("mode", "invalid"))
    sc = int(pf["step_count"]) if "step_count" in pf else 0
    sum_pf = pf.get("summary") if isinstance(pf.get("summary"), dict) else {}
    err_raw = sum_pf.get("error")
    err = None if err_raw is None else str(err_raw)

    if not pf.get("ok"):
        emsg = err if err is not None else "unknown error"
        return {
            "ok": False,
            "mode": mode,
            "overall_outcome": "error",
            "step_count": sc,
            "lines": [
                f"Mode: {mode}",
                f"Preflight failed: {emsg}",
            ],
            "summary": {"message": "", "error": emsg},
        }

    nsteps: List[Dict[str, str]] = list(pf.get("normalized_steps") or [])
    lines = [
        f"Mode: {mode}",
        f"Steps: {len(nsteps)}",
    ]
    for i, st in enumerate(nsteps):
        lines.append(f"Step {i}: {st['robot_id']} -> {st['location_name']}")

    opts = pf.get("options") if isinstance(pf.get("options"), dict) else {}
    if mode == "parallel":
        mw = opts.get("max_workers")
        if mw is not None:
            lines.append(f"Max workers: {mw}")
        msg = "Preview generated for parallel mission."
    else:
        cof = bool(opts.get("continue_on_failure", False))
        lines.append(f"Continue on failure: {str(cof).lower()}")
        msg = "Preview generated for sequence mission."

    return {
        "ok": True,
        "mode": mode,
        "overall_outcome": "success",
        "step_count": sc,
        "lines": lines,
        "summary": {"message": msg, "error": None},
    }


def inspect_team_named_mission_spec(spec: Any) -> Dict[str, Any]:
    """Offline inspection: compact preflight + preview for one mission spec (no ROS)."""
    pf = preflight_team_named_mission_spec(spec)
    pv = preview_team_named_mission_spec(spec)

    pf_ok = bool(pf.get("ok"))
    pv_ok = bool(pv.get("ok"))
    top_ok = pf_ok and pv_ok

    pf_sum = pf.get("summary") if isinstance(pf.get("summary"), dict) else {}
    pv_sum = pv.get("summary") if isinstance(pv.get("summary"), dict) else {}
    pf_err = pf_sum.get("error")
    pv_err = pv_sum.get("error")

    preflight_compact = {
        "ok": pf_ok,
        "overall_outcome": str(pf.get("overall_outcome", "error")),
        "summary": {
            "message": str(pf_sum.get("message", "")),
            "error": None if pf_err is None else str(pf_err),
        },
    }

    raw_lines = pv.get("lines")
    preview_compact = {
        "ok": pv_ok,
        "overall_outcome": str(pv.get("overall_outcome", "error")),
        "lines": list(raw_lines) if isinstance(raw_lines, list) else [],
        "summary": {
            "message": str(pv_sum.get("message", "")),
            "error": None if pv_err is None else str(pv_err),
        },
    }

    mode = str(pf.get("mode", pv.get("mode", "invalid")))
    try:
        step_count = int(pf["step_count"]) if "step_count" in pf else int(pv["step_count"]) if "step_count" in pv else 0
    except (TypeError, ValueError):
        step_count = 0

    if top_ok:
        top_msg = "Mission spec inspection completed successfully."
        top_err = None
    else:
        top_msg = "Mission spec inspection failed."
        if not pf_ok:
            top_err = None if pf_err is None else str(pf_err)
        else:
            top_err = None if pv_err is None else str(pv_err)

    return {
        "ok": top_ok,
        "mode": mode,
        "overall_outcome": "success" if top_ok else "error",
        "step_count": step_count,
        "preflight": preflight_compact,
        "preview": preview_compact,
        "summary": {"message": top_msg, "error": top_err},
    }


def inspect_team_named_mission_specs(specs: Any) -> Dict[str, Any]:
    """Offline inspection for several mission specs in order (no ROS)."""
    if not isinstance(specs, list):
        return {
            "ok": False,
            "overall_outcome": "error",
            "total_specs": 0,
            "ok_count": 0,
            "error_count": 0,
            "results": [],
            "summary": {"message": "", "error": "specs must be a list"},
        }

    results: List[Dict[str, Any]] = []
    ok_count = 0
    error_count = 0
    for i, spec in enumerate(specs):
        ins = inspect_team_named_mission_spec(spec)
        iok = bool(ins.get("ok"))
        if iok:
            ok_count += 1
        else:
            error_count += 1
        try:
            sc = int(ins["step_count"])
        except (KeyError, TypeError, ValueError):
            sc = 0
        pf = ins.get("preflight") if isinstance(ins.get("preflight"), dict) else {}
        pv = ins.get("preview") if isinstance(ins.get("preview"), dict) else {}
        sm = ins.get("summary") if isinstance(ins.get("summary"), dict) else {}
        se = sm.get("error")
        results.append(
            {
                "index": i,
                "ok": iok,
                "mode": str(ins.get("mode", "invalid")),
                "overall_outcome": str(ins.get("overall_outcome", "error")),
                "step_count": sc,
                "preflight": pf,
                "preview": pv,
                "summary": {
                    "message": str(sm.get("message", "")),
                    "error": None if se is None else str(se),
                },
            }
        )

    n = len(specs)
    all_ok = error_count == 0
    if all_ok:
        msg = (
            "Inspection completed successfully for 1 mission spec."
            if n == 1
            else f"Inspection completed successfully for {n} mission specs."
        )
    else:
        msg = (
            "Inspection completed with 1 invalid mission spec."
            if error_count == 1
            else f"Inspection completed with {error_count} invalid mission specs."
        )

    return {
        "ok": all_ok,
        "overall_outcome": "success" if all_ok else "error",
        "total_specs": n,
        "ok_count": ok_count,
        "error_count": error_count,
        "results": results,
        "summary": {"message": msg, "error": None},
    }


def preview_team_named_mission_specs(specs: Any) -> Dict[str, Any]:
    """Human-readable previews for several mission specs in order (no ROS)."""
    if not isinstance(specs, list):
        return {
            "ok": False,
            "overall_outcome": "error",
            "total_specs": 0,
            "ok_count": 0,
            "error_count": 0,
            "results": [],
            "summary": {"message": "", "error": "specs must be a list"},
        }

    results: List[Dict[str, Any]] = []
    ok_count = 0
    error_count = 0
    for i, spec in enumerate(specs):
        pv = preview_team_named_mission_spec(spec)
        pok = bool(pv.get("ok"))
        if pok:
            ok_count += 1
        else:
            error_count += 1
        s = pv.get("summary") if isinstance(pv.get("summary"), dict) else {}
        err = s.get("error")
        raw_lines = pv.get("lines")
        lines = list(raw_lines) if isinstance(raw_lines, list) else []
        results.append(
            {
                "index": i,
                "ok": pok,
                "mode": str(pv.get("mode", "invalid")),
                "overall_outcome": str(pv.get("overall_outcome", "error")),
                "step_count": int(pv["step_count"]) if "step_count" in pv else 0,
                "lines": lines,
                "summary": {
                    "message": str(s.get("message", "")),
                    "error": None if err is None else str(err),
                },
            }
        )

    n = len(specs)
    all_ok = error_count == 0
    if all_ok:
        msg = (
            "Preview generated for 1 mission spec."
            if n == 1
            else f"Preview generated for {n} mission specs."
        )
    else:
        msg = (
            "Preview completed with 1 invalid mission spec."
            if error_count == 1
            else f"Preview completed with {error_count} invalid mission specs."
        )

    return {
        "ok": all_ok,
        "overall_outcome": "success" if all_ok else "error",
        "total_specs": n,
        "ok_count": ok_count,
        "error_count": error_count,
        "results": results,
        "summary": {"message": msg, "error": None},
    }


def summarize_sequence_result(summary: Dict[str, Any]) -> Dict[str, Any]:
    """Compact coordinator-facing view of ``assign_named_sequence`` output (pure, no ROS)."""

    def _invalid(message: str) -> Dict[str, Any]:
        return {
            "ok": False,
            "overall_outcome": "error",
            "mission_state": "invalid",
            "total_steps": 0,
            "steps_run": 0,
            "succeeded_count": 0,
            "failed_count": 0,
            "stopped_early": False,
            "continue_on_failure": False,
            "failed_step_indices": [],
            "succeeded_step_indices": [],
            "first_failed_step_index": None,
            "last_step_index_run": None,
            "message": message,
        }

    if not isinstance(summary, dict):
        return _invalid("Invalid sequence summary.")

    oo_raw = summary.get("overall_outcome")
    if oo_raw not in ("success", "failure"):
        return _invalid("Invalid sequence summary.")

    steps_raw = summary.get("steps")
    if not isinstance(steps_raw, list):
        return _invalid("Invalid sequence summary.")

    try:
        total_steps = int(summary["total_steps"])
        steps_run = int(summary["steps_run"])
        succeeded_count = int(summary["succeeded_count"])
        failed_count = int(summary["failed_count"])
    except (KeyError, TypeError, ValueError):
        return _invalid("Invalid sequence summary.")

    stopped_early = bool(summary.get("stopped_early", False))
    continue_on_failure = bool(summary.get("continue_on_failure", False))

    failed_step_indices: List[int] = []
    succeeded_step_indices: List[int] = []
    for entry in steps_raw:
        if not isinstance(entry, dict):
            return _invalid("Invalid sequence summary.")
        if "index" not in entry or "step_outcome" not in entry:
            return _invalid("Invalid sequence summary.")
        try:
            idx = int(entry["index"])
        except (TypeError, ValueError):
            return _invalid("Invalid sequence summary.")
        soc = str(entry["step_outcome"]).lower()
        if soc == "succeeded":
            succeeded_step_indices.append(idx)
        else:
            failed_step_indices.append(idx)

    failed_step_indices.sort()
    succeeded_step_indices.sort()

    if failed_step_indices or succeeded_step_indices:
        last_step_index_run = max(failed_step_indices + succeeded_step_indices)
    else:
        last_step_index_run = None

    first_failed_step_index = failed_step_indices[0] if failed_step_indices else None

    if oo_raw == "success":
        overall_outcome = "success"
        mission_state = "completed"
        message = "Mission completed successfully."
    else:
        overall_outcome = "failure"
        if stopped_early and failed_step_indices:
            mission_state = "failed_fast"
            n = (first_failed_step_index + 1) if first_failed_step_index is not None else 0
            message = f"Mission failed at step {n} and stopped early."
        else:
            mission_state = "partial_failure"
            if not failed_step_indices:
                message = "Mission did not execute any steps."
            elif len(failed_step_indices) == 1:
                message = "Mission completed with 1 failed step."
            else:
                message = f"Mission completed with {len(failed_step_indices)} failed steps."

    return {
        "ok": overall_outcome == "success",
        "overall_outcome": overall_outcome,
        "mission_state": mission_state,
        "total_steps": total_steps,
        "steps_run": steps_run,
        "succeeded_count": succeeded_count,
        "failed_count": failed_count,
        "stopped_early": stopped_early,
        "continue_on_failure": continue_on_failure,
        "failed_step_indices": list(failed_step_indices),
        "succeeded_step_indices": list(succeeded_step_indices),
        "first_failed_step_index": first_failed_step_index,
        "last_step_index_run": last_step_index_run,
        "message": message,
    }


def summarize_team_named_mission_result(result: Any) -> Dict[str, Any]:
    """Compact interpretation of ``run_team_named_mission_spec`` output (pure, no ROS)."""

    def _inv() -> Dict[str, Any]:
        return {
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

    if not isinstance(result, dict) or "mode" not in result:
        return _inv()
    if result.get("overall_outcome") not in ("success", "failure"):
        return _inv()
    summary = result.get("summary")
    if not isinstance(summary, dict):
        return _inv()

    mode = str(result["mode"]).strip().lower()
    if mode == "sequence":
        sq = summarize_sequence_result(summary)
        if sq.get("overall_outcome") == "error" or sq.get("mission_state") == "invalid":
            return _inv()
        ms = str(sq["mission_state"])
        if ms == "completed":
            msg = "Sequence mission completed successfully."
        elif ms == "failed_fast":
            ffi = sq.get("first_failed_step_index")
            n = (int(ffi) + 1) if ffi is not None else 0
            msg = f"Sequence mission failed at step {n} and stopped early."
        else:
            fc = len(sq["failed_step_indices"])
            if fc == 0:
                msg = "Sequence mission did not execute any steps."
            elif fc == 1:
                msg = "Sequence mission completed with 1 failed step."
            else:
                msg = f"Sequence mission completed with {fc} failed steps."
        return {
            "ok": bool(sq["ok"]),
            "mode": "sequence",
            "overall_outcome": str(sq["overall_outcome"]),
            "mission_state": ms,
            "total_steps": int(sq["total_steps"]),
            "steps_run": int(sq["steps_run"]),
            "succeeded_count": int(sq["succeeded_count"]),
            "failed_count": int(sq["failed_count"]),
            "stopped_early": bool(sq["stopped_early"]),
            "continue_on_failure": bool(sq["continue_on_failure"]),
            "failed_step_indices": list(sq["failed_step_indices"]),
            "succeeded_step_indices": list(sq["succeeded_step_indices"]),
            "first_failed_step_index": sq.get("first_failed_step_index"),
            "last_step_index_run": sq.get("last_step_index_run"),
            "message": msg,
        }

    if mode != "parallel":
        return _inv()

    oo = summary.get("overall_outcome")
    if oo not in ("success", "failure"):
        return _inv()
    steps_raw = summary.get("steps")
    if not isinstance(steps_raw, list):
        return _inv()
    try:
        total_steps = int(summary["total_steps"])
        steps_run = int(summary["steps_run"])
        succeeded_count = int(summary["succeeded_count"])
        failed_count = int(summary["failed_count"])
    except (KeyError, TypeError, ValueError):
        return _inv()

    failed_step_indices: List[int] = []
    succeeded_step_indices: List[int] = []
    for entry in steps_raw:
        if not isinstance(entry, dict) or "index" not in entry or "step_outcome" not in entry:
            return _inv()
        try:
            idx = int(entry["index"])
        except (TypeError, ValueError):
            return _inv()
        if str(entry["step_outcome"]).lower() == "succeeded":
            succeeded_step_indices.append(idx)
        else:
            failed_step_indices.append(idx)

    failed_step_indices.sort()
    succeeded_step_indices.sort()

    if failed_step_indices or succeeded_step_indices:
        last_step_index_run = max(failed_step_indices + succeeded_step_indices)
    else:
        last_step_index_run = None

    first_failed_step_index = failed_step_indices[0] if failed_step_indices else None

    if not failed_step_indices and oo == "success":
        p_ms = "completed"
        msg = "Parallel mission completed successfully."
    else:
        p_ms = "partial_failure"
        fc = len(failed_step_indices)
        if fc == 1:
            msg = "Parallel mission completed with 1 failed step."
        elif fc == 0:
            msg = "Parallel mission did not execute any steps."
        else:
            msg = f"Parallel mission completed with {fc} failed steps."

    return {
        "ok": oo == "success",
        "mode": "parallel",
        "overall_outcome": str(oo),
        "mission_state": p_ms,
        "total_steps": total_steps,
        "steps_run": steps_run,
        "succeeded_count": succeeded_count,
        "failed_count": failed_count,
        "stopped_early": None,
        "continue_on_failure": None,
        "failed_step_indices": list(failed_step_indices),
        "succeeded_step_indices": list(succeeded_step_indices),
        "first_failed_step_index": first_failed_step_index,
        "last_step_index_run": last_step_index_run,
        "message": msg,
    }


def execute_team_named_mission_spec(spec: Any) -> Dict[str, Any]:
    """Run one mission spec and attach a compact execution summary (ROS via ``run_team_named_mission_spec``)."""
    run = run_team_named_mission_spec(spec)
    ex = summarize_team_named_mission_result(run)

    run_ok = bool(run.get("ok"))
    ex_ok = bool(ex.get("ok"))
    top_ok = run_ok and ex_ok

    rs = run.get("summary") if isinstance(run.get("summary"), dict) else {}
    run_compact = {
        "ok": run_ok,
        "mode": str(run.get("mode", "invalid")),
        "overall_outcome": str(run.get("overall_outcome", "error")),
        "summary": rs,
    }

    if top_ok:
        top_oo = "success"
    else:
        top_oo = str(ex.get("overall_outcome", run.get("overall_outcome", "error")))

    em = ex.get("mode")
    mode = str(em) if em not in (None, "") else str(run.get("mode") or "invalid")

    if top_ok:
        top_msg = "Mission spec executed successfully."
        top_err = None
    else:
        top_msg = ""
        if ex.get("overall_outcome") == "error" or ex.get("mission_state") == "invalid":
            top_err = str(ex.get("message") or "Invalid team mission result.")
        else:
            m = ex.get("message")
            top_err = str(m) if m else None

    return {
        "ok": top_ok,
        "mode": mode,
        "overall_outcome": top_oo,
        "run_result": run_compact,
        "execution_summary": ex,
        "summary": {"message": top_msg, "error": top_err},
    }
