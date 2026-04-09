"""Layer B: minimal team-facing coordinator over the validated execution stack."""

from __future__ import annotations

import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List, Optional, Tuple

from ..agent.mission_agent_facade import MissionAgentFacade
from ..agent.navigate_failure_classification_v51 import (
    NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE,
    navigate_failure_kind,
)
from ..agent.policy_config import build_default_policy_config
from ..agent.sequence_utils import _navigate_named_command, _navigate_ok, _validate_steps
from ..agent.wait_utils import wait_for_terminal_navigation_state


def _validate_coordinator_sequence_steps(steps: Any) -> Optional[str]:
    """
    Sequence steps: robot_id + location_name (via shared list checks) plus optional
    ``alternate_location_name`` (V7.1): non-empty, distinct from primary, allowlisted.
    """
    base = _validate_steps(steps)
    if base is not None:
        return base
    allowed = build_default_policy_config().allowed_locations
    assert isinstance(steps, list)
    for i, raw in enumerate(steps):
        assert isinstance(raw, dict)
        loc = str(raw.get("location_name", "")).strip()
        alt_raw = raw.get("alternate_location_name", None)
        if alt_raw is None:
            continue
        if not isinstance(alt_raw, str):
            return f"step {i} alternate_location_name must be a string"
        a = alt_raw.strip()
        if not a:
            continue
        if a == loc:
            return f"step {i} alternate_location_name must differ from location_name"
        if a not in allowed:
            return f"step {i} alternate_location_name {a!r} is not permitted"
    return None


def cancel_navigation_via_facade(
    facade: MissionAgentFacade,
    robot_id: str,
    goal_id: str,
) -> Dict[str, Any]:
    """
    Forward cancel through ``facade.handle_command`` only.

    No coordinator registry; bridge remains ownership authority. Returns the raw
    result dict (e.g. wrong_robot vs owned cancel) without reinterpretation.
    """
    cmd = {
        "type": "cancel",
        "target": "navigation",
        "robot_id": str(robot_id).strip(),
        "goal_id": str(goal_id).strip(),
    }
    return facade.handle_command(cmd)


def cancel_navigation_with_ros(
    robot_id: str,
    goal_id: str,
    *,
    bridge_node_name: str = "mission_bridge_node",
) -> Dict[str, Any]:
    """
    Layer B cancel entrypoint when no shared ``MissionAgentFacade`` exists.

    Opens a ROS facade, delegates to ``cancel_navigation_via_facade``, then closes.
    Bridge remains ownership authority (wrong_robot vs owned cancel).
    """
    facade: MissionAgentFacade | None = None
    try:
        facade = MissionAgentFacade.with_ros(bridge_node_name=bridge_node_name)
        return cancel_navigation_via_facade(facade, robot_id, goal_id)
    finally:
        if facade is not None:
            try:
                facade.close()
            except Exception:
                pass


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
            leg = {
                "robot_id": rid,
                "location_name": loc,
                "goal_id": "",
                "outcome": "failed",
                "status": str(nav.get("status", "")),
                "nav_status": str(nav.get("nav_status", "")),
                "message": str(nav.get("message", "") or reason),
                "elapsed_sec": _elapsed(),
            }
            k = navigate_failure_kind(nav)
            if k is not None:
                leg["navigate_failure_kind"] = k
            return leg

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
    steps: List[Dict[str, Any]],
    *,
    per_goal_timeout_sec: float = 120.0,
    poll_interval_sec: float = 1.0,
    continue_on_failure: bool = False,
    hard_stop_on_advisory_blocked: bool = False,
    bridge_node_name: str = "mission_bridge_node",
) -> Dict[str, Any]:
    """
    Sequential team mission: each step is one ``assign_named_navigation`` leg (own facade per leg).

    **V7.1:** Optional ``alternate_location_name`` per step: after a primary failure with
    ``navigate_failure_kind == advisory_blocked_passage``, the coordinator issues exactly one
    navigate to the alternate (same ``robot_id``), then uses that result as the leg outcome for
    success/failure and V6.1 hard-stop evaluation.

    When ``hard_stop_on_advisory_blocked`` is true, evaluation uses the **effective** leg result
    (after any alternate attempt), then the sequence may stop per V6.1.
    """
    err = _validate_coordinator_sequence_steps(steps)
    hard = bool(hard_stop_on_advisory_blocked)
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
            "hard_stop_on_advisory_blocked": hard,
            "stopped_due_to_advisory_blocked": False,
            "steps": [],
            "error": err,
        }

    total = len(steps)
    out_steps: List[Dict[str, Any]] = []
    succeeded_count = 0
    failed_count = 0
    cont = bool(continue_on_failure)
    stopped_due_to_advisory_blocked = False

    for idx, step in enumerate(steps):
        rid = str(step["robot_id"]).strip()
        loc = str(step["location_name"]).strip()
        alt_raw = step.get("alternate_location_name")
        alt = (
            str(alt_raw).strip()
            if isinstance(alt_raw, str) and str(alt_raw).strip()
            else ""
        )

        primary = assign_named_navigation(
            rid,
            loc,
            per_goal_timeout_sec=float(per_goal_timeout_sec),
            poll_interval_sec=float(poll_interval_sec),
            bridge_node_name=bridge_node_name,
        )
        alternate_attempted = False
        alternate_leg: Optional[Dict[str, Any]] = None
        effective = primary
        if (
            str(primary.get("outcome", "")).lower() != "succeeded"
            and primary.get("navigate_failure_kind")
            == NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE
            and alt
        ):
            alternate_attempted = True
            alternate_leg = assign_named_navigation(
                rid,
                alt,
                per_goal_timeout_sec=float(per_goal_timeout_sec),
                poll_interval_sec=float(poll_interval_sec),
                bridge_node_name=bridge_node_name,
            )
            effective = alternate_leg

        oc = str(effective.get("outcome", "")).lower()
        step_ok = oc == "succeeded"
        if step_ok:
            succeeded_count += 1
            step_outcome = "succeeded"
        else:
            failed_count += 1
            step_outcome = "failed"

        rec: Dict[str, Any] = {
            "index": idx,
            "robot_id": rid,
            "location_name": loc,
            "result": effective,
            "step_outcome": step_outcome,
        }
        if alt:
            rec["alternate_location_name"] = alt
            rec["primary_result"] = primary
            rec["alternate_attempted"] = alternate_attempted
            rec["alternate_result"] = alternate_leg
        out_steps.append(rec)

        if not step_ok and not cont:
            break
        if (
            not step_ok
            and hard
            and effective.get("navigate_failure_kind")
            == NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE
        ):
            stopped_due_to_advisory_blocked = True
            break

    steps_run = len(out_steps)
    stopped_early = steps_run < total and failed_count > 0 and (
        not cont or stopped_due_to_advisory_blocked
    )
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
        "hard_stop_on_advisory_blocked": hard,
        "stopped_due_to_advisory_blocked": stopped_due_to_advisory_blocked,
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
    allowed = build_default_policy_config().allowed_locations
    norm_steps: List[Dict[str, Any]] = []
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
        entry: Dict[str, Any] = {"robot_id": rs, "location_name": ls}
        alt_raw = raw.get("alternate_location_name", None)
        if alt_raw is not None:
            if not isinstance(alt_raw, str):
                return {
                    "ok": False,
                    "mode": m,
                    "steps": [],
                    "error": f"step {i} alternate_location_name must be a string",
                }
            a = alt_raw.strip()
            if a:
                if a == ls:
                    return {
                        "ok": False,
                        "mode": m,
                        "steps": [],
                        "error": (
                            f"step {i} alternate_location_name must differ from location_name"
                        ),
                    }
                if a not in allowed:
                    return {
                        "ok": False,
                        "mode": m,
                        "steps": [],
                        "error": (
                            f"step {i} alternate_location_name {a!r} is not permitted"
                        ),
                    }
                entry["alternate_location_name"] = a
        norm_steps.append(entry)
    return {
        "ok": True,
        "mode": m,
        "steps": norm_steps,
        "error": None,
    }


def validate_team_named_mission_spec_contract(spec: Any) -> Dict[str, Any]:
    """Lightweight team mission spec v1 contract: shape, version, and mode support only (pure, no ROS)."""
    bad = "Mission spec v1 contract violation."
    good = "Mission spec satisfies v1 contract."
    if not isinstance(spec, dict):
        return {
            "ok": False,
            "overall_outcome": "error",
            "message": bad,
            "errors": ["spec must be a dict"],
        }

    errors: List[str] = []
    if spec.get("version") != "v1":
        errors.append('version must be exactly "v1"')

    if "mode" not in spec:
        errors.append("missing mode")
        mnorm = ""
    else:
        mnorm = str(spec["mode"]).strip().lower()
        if mnorm not in ("sequence", "parallel"):
            errors.append("mode must be sequence or parallel")

    if "steps" not in spec:
        errors.append("missing steps")
        steps_raw: Any = None
    else:
        steps_raw = spec["steps"]
        if not isinstance(steps_raw, list):
            errors.append("steps must be a list")
            steps_raw = []

    if "options" in spec and not isinstance(spec["options"], dict):
        errors.append("options must be a dict when present")

    allowed_loc = build_default_policy_config().allowed_locations
    if isinstance(steps_raw, list):
        seen_parallel: set[str] = set()
        for i, raw_step in enumerate(steps_raw):
            if not isinstance(raw_step, dict):
                errors.append(f"step {i} must be a dict")
                continue
            rid = raw_step.get("robot_id")
            loc = raw_step.get("location_name")
            rs = "" if rid is None else str(rid).strip()
            ls = "" if loc is None else str(loc).strip()
            if not rs:
                errors.append(f"step {i} has empty robot_id")
            if not ls:
                errors.append(f"step {i} has empty location_name")
            alt_raw = raw_step.get("alternate_location_name", None)
            if alt_raw is not None:
                if not isinstance(alt_raw, str):
                    errors.append(f"step {i} alternate_location_name must be a string")
                else:
                    a = alt_raw.strip()
                    if a:
                        if a == ls:
                            errors.append(
                                f"step {i} alternate_location_name must differ from location_name"
                            )
                        elif a not in allowed_loc:
                            errors.append(
                                f"step {i} alternate_location_name {a!r} is not permitted"
                            )
            if mnorm == "parallel" and rs:
                if rs in seen_parallel:
                    errors.append("duplicate robot_id in parallel spec")
                seen_parallel.add(rs)

    if errors:
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": errors}
    return {"ok": True, "overall_outcome": "success", "message": good, "errors": []}


def validate_team_named_mission_specs_contract(specs: Any) -> Dict[str, Any]:
    """Batch mission spec v1 contract: each item must satisfy ``validate_team_named_mission_spec_contract``."""
    bad = "Mission specs batch v1 contract violation."
    good = "All mission specs satisfy v1 contract."
    if not isinstance(specs, list):
        return {
            "ok": False,
            "overall_outcome": "error",
            "message": bad,
            "errors": ["specs must be a list"],
            "spec_count": 0,
        }

    all_errors: List[str] = []
    for i, item in enumerate(specs):
        if not isinstance(item, dict):
            all_errors.append(f"spec[{i}]: spec must be a dict")
            continue
        r = validate_team_named_mission_spec_contract(item)
        if not r.get("ok"):
            for e in r.get("errors", []):
                all_errors.append(f"spec[{i}]: {e}")

    n = len(specs)
    if all_errors:
        return {
            "ok": False,
            "overall_outcome": "error",
            "message": bad,
            "errors": all_errors,
            "spec_count": n,
        }
    return {
        "ok": True,
        "overall_outcome": "success",
        "message": good,
        "errors": [],
        "spec_count": n,
    }


def assign_team_named_mission(
    steps: Any,
    mode: str = "sequence",
    *,
    per_goal_timeout_sec: float = 120.0,
    poll_interval_sec: float = 1.0,
    continue_on_failure: bool = False,
    hard_stop_on_advisory_blocked: bool = False,
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
            hard_stop_on_advisory_blocked=bool(hard_stop_on_advisory_blocked),
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
        "hard_stop_on_advisory_blocked",
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
        hard_stop_on_advisory_blocked=bool(opts.get("hard_stop_on_advisory_blocked", False)),
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
            "version": "v1",
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
        "version": "v1",
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
        "version": "v1",
        "overall_outcome": top_oo,
        "run_result": run_compact,
        "execution_summary": ex,
        "summary": {"message": top_msg, "error": top_err},
    }


def execute_team_named_mission_specs_checked(
    specs: Any,
    continue_on_batch_failure: bool = False,
) -> Dict[str, Any]:
    """Run several mission specs, summarize, and validate batch summary invariants (ROS via ``execute_team_named_mission_specs``)."""
    exe = execute_team_named_mission_specs(specs, continue_on_batch_failure=continue_on_batch_failure)
    es = exe.get("execution_summary")
    val = validate_team_named_mission_specs_summary(es if isinstance(es, dict) else {})

    exe_ok = bool(exe.get("ok"))
    val_ok = bool(val.get("ok"))
    top_ok = exe_ok and val_ok

    if top_ok:
        top_oo = "success"
        top_msg = "Batch mission specs executed and validated successfully."
        top_err = None
    else:
        top_msg = ""
        if val.get("overall_outcome") == "error":
            top_oo = "error"
        else:
            top_oo = str(exe.get("overall_outcome", "error"))

        if not val_ok:
            top_err = str(val.get("message", "")) if val.get("message") else None
        else:
            esum = exe.get("summary") if isinstance(exe.get("summary"), dict) else {}
            if esum.get("error") is not None:
                top_err = str(esum["error"])
            else:
                inner = exe.get("execution_summary") if isinstance(exe.get("execution_summary"), dict) else {}
                im = inner.get("message")
                top_err = str(im) if im else None

    return {
        "ok": top_ok,
        "version": "v1",
        "overall_outcome": top_oo,
        "execution": exe,
        "validation": val,
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
            "hard_stop_on_advisory_blocked",
            "max_workers",
            "bridge_node_name",
        )
        picked = {k: raw[k] for k in keys if k in raw}
        return {
            "per_goal_timeout_sec": picked.get("per_goal_timeout_sec", 120.0),
            "poll_interval_sec": picked.get("poll_interval_sec", 1.0),
            "continue_on_failure": picked.get("continue_on_failure", False),
            "hard_stop_on_advisory_blocked": bool(
                picked.get("hard_stop_on_advisory_blocked", False)
            ),
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
        hs = bool(opts.get("hard_stop_on_advisory_blocked", False))
        lines.append(f"Hard stop on advisory blocked: {str(hs).lower()}")
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
        "version": "v1",
        "mode": mode,
        "overall_outcome": "success" if top_ok else "error",
        "step_count": step_count,
        "preflight": preflight_compact,
        "preview": preview_compact,
        "summary": {"message": top_msg, "error": top_err},
    }


def validate_team_named_mission_inspection(inspection: Any) -> Dict[str, Any]:
    """Check internal consistency of ``inspect_team_named_mission_spec`` output (pure, no ROS)."""
    good = "Mission inspection is internally consistent."
    bad = "Mission inspection is invalid."
    if not isinstance(inspection, dict):
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": ["inspection must be a dict"]}

    req_top = ("ok", "mode", "overall_outcome", "step_count", "preflight", "preview", "summary")
    errors: List[str] = []
    for k in req_top:
        if k not in inspection:
            errors.append(f"missing {k}")

    sc_raw = inspection.get("step_count")
    if not isinstance(sc_raw, int) or isinstance(sc_raw, bool):
        errors.append("step_count must be an integer")

    mode_s = str(inspection.get("mode", "")).strip().lower()
    if mode_s not in ("sequence", "parallel", "invalid"):
        errors.append("mode must be sequence, parallel, or invalid")

    oo = inspection.get("overall_outcome")
    iok = inspection.get("ok")
    if oo == "success" and iok is not True:
        errors.append("overall_outcome success requires ok true")
    if iok is True and oo != "success":
        errors.append("ok true requires overall_outcome success")

    pf = inspection.get("preflight")
    pv = inspection.get("preview")
    pv_lines: Any = None
    if not isinstance(pf, dict):
        errors.append("preflight must be a dict")
    if not isinstance(pv, dict):
        errors.append("preview must be a dict")

    if isinstance(pf, dict):
        for k in ("ok", "overall_outcome", "summary"):
            if k not in pf:
                errors.append(f"preflight missing {k}")
        pf_sum = pf.get("summary") if isinstance(pf.get("summary"), dict) else None
        if pf_sum is None:
            errors.append("preflight.summary must be a dict")
        elif str(pf.get("overall_outcome")) == "success" and pf_sum.get("error") is not None:
            errors.append("preflight success requires summary.error null")
        if iok is True and pf.get("ok") is not True:
            errors.append("inspection ok requires preflight.ok true")

    if isinstance(pv, dict):
        for k in ("ok", "overall_outcome", "lines", "summary"):
            if k not in pv:
                errors.append(f"preview missing {k}")
        pv_lines = pv.get("lines")
        if not isinstance(pv_lines, list):
            errors.append("preview.lines must be a list")
        pv_sum = pv.get("summary") if isinstance(pv.get("summary"), dict) else None
        if pv_sum is None:
            errors.append("preview.summary must be a dict")
        elif str(pv.get("overall_outcome")) == "success" and pv_sum.get("error") is not None:
            errors.append("preview success requires summary.error null")
        if iok is True and pv.get("ok") is not True:
            errors.append("inspection ok requires preview.ok true")

    if (
        iok is True
        and isinstance(pv_lines, list)
        and isinstance(sc_raw, int)
        and not isinstance(sc_raw, bool)
        and sc_raw == 0
    ):
        has_mode_ln = any(isinstance(ln, str) and ln.startswith("Mode:") for ln in pv_lines)
        has_steps_ln = any(isinstance(ln, str) and ln.startswith("Steps:") for ln in pv_lines)
        if not has_mode_ln or not has_steps_ln:
            errors.append("step_count 0 with ok true requires Mode and Steps preview lines")

    if errors:
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": errors}
    return {"ok": True, "overall_outcome": "success", "message": good, "errors": []}


def inspect_team_named_mission_spec_checked(spec: Any) -> Dict[str, Any]:
    """Inspect one mission spec and validate inspection invariants (no ROS)."""
    ins = inspect_team_named_mission_spec(spec)
    val = validate_team_named_mission_inspection(ins if isinstance(ins, dict) else {})

    ins_ok = bool(ins.get("ok")) if isinstance(ins, dict) else False
    val_ok = bool(val.get("ok"))
    top_ok = ins_ok and val_ok

    if top_ok:
        top_oo = "success"
        top_msg = "Mission spec inspected and validated successfully."
        top_err = None
    else:
        top_oo = "error"
        top_msg = ""
        if not val_ok:
            top_err = str(val.get("message", "")) if val.get("message") else None
        else:
            isum = ins.get("summary") if isinstance(ins, dict) and isinstance(ins.get("summary"), dict) else {}
            e = isum.get("error")
            top_err = str(e) if e is not None else None

    mode = str(ins.get("mode", "invalid")) if isinstance(ins, dict) else "invalid"

    return {
        "ok": top_ok,
        "version": "v1",
        "mode": mode,
        "overall_outcome": top_oo,
        "inspection": ins if isinstance(ins, dict) else {},
        "validation": val,
        "summary": {"message": top_msg, "error": top_err},
    }


def inspect_team_named_mission_specs(specs: Any) -> Dict[str, Any]:
    """Offline inspection for several mission specs in order (no ROS)."""
    if not isinstance(specs, list):
        return {
            "ok": False,
            "version": "v1",
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
        "version": "v1",
        "overall_outcome": "success" if all_ok else "error",
        "total_specs": n,
        "ok_count": ok_count,
        "error_count": error_count,
        "results": results,
        "summary": {"message": msg, "error": None},
    }


def validate_team_named_mission_specs_inspection(batch_inspection: Any) -> Dict[str, Any]:
    """Check internal consistency of ``inspect_team_named_mission_specs`` output (pure, no ROS)."""
    good = "Batch mission inspection is internally consistent."
    bad = "Batch mission inspection is invalid."
    if not isinstance(batch_inspection, dict):
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": ["batch_inspection must be a dict"]}

    req_top = ("ok", "overall_outcome", "total_specs", "ok_count", "error_count", "results", "summary")
    errors: List[str] = []
    for k in req_top:
        if k not in batch_inspection:
            errors.append(f"missing {k}")

    results = batch_inspection.get("results")
    if not isinstance(results, list):
        errors.append("results must be a list")

    ts = batch_inspection.get("total_specs")
    oc = batch_inspection.get("ok_count")
    ec = batch_inspection.get("error_count")
    for name, raw in (("total_specs", ts), ("ok_count", oc), ("error_count", ec)):
        if not isinstance(raw, int) or isinstance(raw, bool):
            errors.append(f"{name} must be a non-bool int")

    if isinstance(ts, int) and not isinstance(ts, bool) and isinstance(oc, int) and not isinstance(oc, bool):
        if isinstance(ec, int) and not isinstance(ec, bool) and oc + ec != ts:
            errors.append("ok_count + error_count must equal total_specs")

    top_ok = batch_inspection.get("ok")
    boo = batch_inspection.get("overall_outcome")
    if boo == "success" and top_ok is not True:
        errors.append("overall_outcome success requires ok true")
    if top_ok is True and isinstance(ec, int) and not isinstance(ec, bool) and ec != 0:
        errors.append("ok true requires error_count==0")

    if (
        isinstance(ts, int)
        and not isinstance(ts, bool)
        and isinstance(ec, int)
        and not isinstance(ec, bool)
        and ts > 0
        and ec == 0
    ):
        if boo != "success":
            errors.append("error_count 0 with total_specs>0 requires overall_outcome success")
        if top_ok is not True:
            errors.append("error_count 0 with total_specs>0 requires ok true")

    if (
        isinstance(ec, int)
        and not isinstance(ec, bool)
        and ec > 0
        and boo == "success"
    ):
        errors.append("error_count>0 is incompatible with overall_outcome success")

    if isinstance(results, list) and isinstance(ts, int) and not isinstance(ts, bool) and len(results) != ts:
        errors.append("len(results) must equal total_specs")

    row_keys = ("index", "ok", "mode", "overall_outcome", "step_count", "preflight", "preview", "summary")
    seen_idx: set[int] = set()
    if isinstance(results, list):
        for ri, row in enumerate(results):
            if not isinstance(row, dict):
                errors.append(f"results[{ri}] must be a dict")
                continue
            for k in row_keys:
                if k not in row:
                    errors.append(f"results[{ri}] missing {k}")
            idx_raw = row.get("index")
            if not isinstance(idx_raw, int) or isinstance(idx_raw, bool):
                errors.append(f"results[{ri}].index must be a non-bool int")
            elif idx_raw in seen_idx:
                errors.append("result indices must be unique")
            else:
                seen_idx.add(idx_raw)

            sc_r = row.get("step_count")
            if not isinstance(sc_r, int) or isinstance(sc_r, bool):
                errors.append(f"results[{ri}].step_count must be a non-bool int")

            mode_s = str(row.get("mode", "")).strip().lower()
            if mode_s not in ("sequence", "parallel", "invalid"):
                errors.append(f"results[{ri}].mode must be sequence, parallel, or invalid")

            rok = row.get("ok")
            roo = row.get("overall_outcome")
            if rok is True and roo != "success":
                errors.append(f"results[{ri}] ok true requires overall_outcome success")

            for sub, sk in (("preflight", "preflight"), ("preview", "preview"), ("summary", "summary")):
                if not isinstance(row.get(sub), dict):
                    errors.append(f"results[{ri}].{sk} must be a dict")

    if errors:
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": errors}
    return {"ok": True, "overall_outcome": "success", "message": good, "errors": []}


def inspect_team_named_mission_specs_checked(specs: Any) -> Dict[str, Any]:
    """Inspect several mission specs and validate batch inspection invariants (no ROS)."""
    ins = inspect_team_named_mission_specs(specs)
    val = validate_team_named_mission_specs_inspection(ins if isinstance(ins, dict) else {})

    ins_ok = bool(ins.get("ok")) if isinstance(ins, dict) else False
    val_ok = bool(val.get("ok"))
    top_ok = ins_ok and val_ok

    if top_ok:
        top_oo = "success"
        top_msg = "Batch mission specs inspected and validated successfully."
        top_err = None
    else:
        top_oo = "error"
        top_msg = ""
        if not val_ok:
            top_err = str(val.get("message", "")) if val.get("message") else None
        else:
            isum = ins.get("summary") if isinstance(ins, dict) and isinstance(ins.get("summary"), dict) else {}
            e = isum.get("error")
            top_err = str(e) if e is not None else None

    return {
        "ok": top_ok,
        "version": "v1",
        "overall_outcome": top_oo,
        "inspection": ins if isinstance(ins, dict) else {},
        "validation": val,
        "summary": {"message": top_msg, "error": top_err},
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
            "version": "v1",
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
            "version": "v1",
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
        "version": "v1",
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
        "version": "v1",
        "mode": mode,
        "overall_outcome": top_oo,
        "run_result": run_compact,
        "execution_summary": ex,
        "summary": {"message": top_msg, "error": top_err},
    }


def execute_team_named_mission_spec_checked(spec: Any) -> Dict[str, Any]:
    """Run one mission spec, summarize, and validate summary invariants (ROS via ``execute_team_named_mission_spec``)."""
    exe = execute_team_named_mission_spec(spec)
    es = exe.get("execution_summary")
    val = validate_team_named_mission_summary(es if isinstance(es, dict) else {})

    exe_ok = bool(exe.get("ok"))
    val_ok = bool(val.get("ok"))
    top_ok = exe_ok and val_ok

    if top_ok:
        top_oo = "success"
        top_msg = "Mission spec executed and validated successfully."
        top_err = None
    else:
        top_msg = ""
        if val.get("overall_outcome") == "error":
            top_oo = "error"
        else:
            top_oo = str(exe.get("overall_outcome", "error"))

        if not val_ok:
            top_err = str(val.get("message", "")) if val.get("message") else None
        else:
            esum = exe.get("summary") if isinstance(exe.get("summary"), dict) else {}
            if esum.get("error") is not None:
                top_err = str(esum["error"])
            else:
                inner = exe.get("execution_summary") if isinstance(exe.get("execution_summary"), dict) else {}
                im = inner.get("message")
                top_err = str(im) if im else None

    return {
        "ok": top_ok,
        "version": "v1",
        "mode": str(exe.get("mode", "invalid")),
        "overall_outcome": top_oo,
        "execution": exe,
        "validation": val,
        "summary": {"message": top_msg, "error": top_err},
    }


def validate_team_named_mission_execution_contract(result: Any) -> Dict[str, Any]:
    """Stable v1 shape for single-spec execute / execute_checked / summarize outputs (pure, no ROS)."""
    bad = "Team mission execution payload v1 contract violation."
    good = "Team mission execution payload satisfies v1 contract."
    if not isinstance(result, dict):
        return {
            "ok": False,
            "overall_outcome": "error",
            "message": bad,
            "errors": ["result must be a dict"],
        }
    if result.get("version") != "v1":
        return {
            "ok": False,
            "overall_outcome": "error",
            "message": bad,
            "errors": ['version must be exactly "v1"'],
        }

    errors: List[str] = []

    def _req(keys: Tuple[str, ...]) -> None:
        for k in keys:
            if k not in result:
                errors.append(f"missing {k}")

    if "execution" in result and "validation" in result:
        _req(("ok", "mode", "overall_outcome", "execution", "validation", "summary", "version"))
        if not isinstance(result.get("execution"), dict):
            errors.append("execution must be a dict")
        if not isinstance(result.get("validation"), dict):
            errors.append("validation must be a dict")
        if not isinstance(result.get("summary"), dict):
            errors.append("summary must be a dict")
    elif "run_result" in result and "execution_summary" in result:
        _req(("ok", "mode", "overall_outcome", "run_result", "execution_summary", "summary", "version"))
        if not isinstance(result.get("run_result"), dict):
            errors.append("run_result must be a dict")
        if not isinstance(result.get("execution_summary"), dict):
            errors.append("execution_summary must be a dict")
        if not isinstance(result.get("summary"), dict):
            errors.append("summary must be a dict")
    elif "mission_state" in result and "failed_step_indices" in result:
        _req(
            (
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
                "message",
                "version",
            )
        )
        if not isinstance(result.get("failed_step_indices"), list):
            errors.append("failed_step_indices must be a list")
        if not isinstance(result.get("succeeded_step_indices"), list):
            errors.append("succeeded_step_indices must be a list")
    else:
        errors.append("unknown payload shape (expected execute, execute_checked, or summarize_team_named_mission_result)")

    if errors:
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": errors}
    return {"ok": True, "overall_outcome": "success", "message": good, "errors": []}


def validate_team_named_mission_specs_execution_contract(result: Any) -> Dict[str, Any]:
    """Stable v1 shape for batch execute / execute_checked / batch summarize outputs (pure, no ROS)."""
    bad = "Batch team mission execution payload v1 contract violation."
    good = "Batch team mission execution payload satisfies v1 contract."
    if not isinstance(result, dict):
        return {
            "ok": False,
            "overall_outcome": "error",
            "message": bad,
            "errors": ["result must be a dict"],
        }
    if result.get("version") != "v1":
        return {
            "ok": False,
            "overall_outcome": "error",
            "message": bad,
            "errors": ['version must be exactly "v1"'],
        }

    errors: List[str] = []

    def _req(keys: Tuple[str, ...]) -> None:
        for k in keys:
            if k not in result:
                errors.append(f"missing {k}")

    if "execution" in result and "validation" in result:
        _req(("ok", "overall_outcome", "execution", "validation", "summary", "version"))
        if not isinstance(result.get("execution"), dict):
            errors.append("execution must be a dict")
        if not isinstance(result.get("validation"), dict):
            errors.append("validation must be a dict")
        if not isinstance(result.get("summary"), dict):
            errors.append("summary must be a dict")
    elif "run_result" in result and "execution_summary" in result:
        _req(("ok", "overall_outcome", "run_result", "execution_summary", "summary", "version"))
        if not isinstance(result.get("run_result"), dict):
            errors.append("run_result must be a dict")
        if not isinstance(result.get("execution_summary"), dict):
            errors.append("execution_summary must be a dict")
        if not isinstance(result.get("summary"), dict):
            errors.append("summary must be a dict")
    elif "mission_state" in result and "failed_spec_indices" in result:
        _req(
            (
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
                "message",
                "version",
            )
        )
        if not isinstance(result.get("failed_spec_indices"), list):
            errors.append("failed_spec_indices must be a list")
        if not isinstance(result.get("succeeded_spec_indices"), list):
            errors.append("succeeded_spec_indices must be a list")
    else:
        errors.append(
            "unknown payload shape (expected execute_team_named_mission_specs, execute_checked, or summarize_team_named_mission_specs_result)"
        )

    if errors:
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": errors}
    return {"ok": True, "overall_outcome": "success", "message": good, "errors": []}


def get_team_named_mission_api_manifest() -> Dict[str, Any]:
    """Static manifest of stable Layer B mission-spec entrypoints (pure, no ROS)."""
    return {
        "ok": True,
        "overall_outcome": "success",
        "api_version": "v1",
        "capabilities": {
            "single_spec": {
                "inspect": True,
                "inspect_checked": True,
                "execute": True,
                "execute_checked": True,
            },
            "batch_specs": {
                "inspect": True,
                "inspect_checked": True,
                "execute": True,
                "execute_checked": True,
            },
        },
        "entrypoints": {
            "single_spec": {
                "inspect": "inspect_team_named_mission_spec",
                "inspect_checked": "inspect_team_named_mission_spec_checked",
                "execute": "execute_team_named_mission_spec",
                "execute_checked": "execute_team_named_mission_spec_checked",
            },
            "batch_specs": {
                "inspect": "inspect_team_named_mission_specs",
                "inspect_checked": "inspect_team_named_mission_specs_checked",
                "execute": "execute_team_named_mission_specs",
                "execute_checked": "execute_team_named_mission_specs_checked",
            },
        },
        "summary": {
            "message": "Layer B mission API manifest generated successfully.",
            "error": None,
        },
    }


def validate_team_named_mission_api_manifest(manifest: Any) -> Dict[str, Any]:
    """Check internal consistency of ``get_team_named_mission_api_manifest`` output (pure, no ROS)."""
    good = "Mission API manifest is internally consistent."
    bad = "Mission API manifest is invalid."
    if not isinstance(manifest, dict):
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": ["manifest must be a dict"]}

    req_top = ("ok", "overall_outcome", "api_version", "capabilities", "entrypoints", "summary")
    errors: List[str] = []
    for k in req_top:
        if k not in manifest:
            errors.append(f"missing {k}")

    if manifest.get("api_version") != "v1":
        errors.append("api_version must be v1")

    oo = manifest.get("overall_outcome")
    mok = manifest.get("ok")
    if oo == "success" and mok is not True:
        errors.append("overall_outcome success requires ok true")

    sm = manifest.get("summary")
    if not isinstance(sm, dict):
        errors.append("summary must be a dict")
    elif oo == "success" and sm.get("error") is not None:
        errors.append("success requires summary.error null")

    cap_inner = ("inspect", "inspect_checked", "execute", "execute_checked")
    cap_groups = ("single_spec", "batch_specs")
    want_single_ep = {
        "inspect": "inspect_team_named_mission_spec",
        "inspect_checked": "inspect_team_named_mission_spec_checked",
        "execute": "execute_team_named_mission_spec",
        "execute_checked": "execute_team_named_mission_spec_checked",
    }
    want_batch_ep = {
        "inspect": "inspect_team_named_mission_specs",
        "inspect_checked": "inspect_team_named_mission_specs_checked",
        "execute": "execute_team_named_mission_specs",
        "execute_checked": "execute_team_named_mission_specs_checked",
    }

    cap = manifest.get("capabilities")
    if not isinstance(cap, dict):
        errors.append("capabilities must be a dict")
    else:
        for g in cap_groups:
            if g not in cap:
                errors.append(f"capabilities missing {g}")
                continue
            cg = cap[g]
            if not isinstance(cg, dict):
                errors.append(f"capabilities.{g} must be a dict")
                continue
            for ck in cap_inner:
                if ck not in cg:
                    errors.append(f"capabilities.{g} missing {ck}")
                elif type(cg[ck]) is not bool:
                    errors.append(f"capabilities.{g}.{ck} must be bool")
                elif cg[ck] is not True:
                    errors.append(f"capabilities.{g}.{ck} must be true")

    ep = manifest.get("entrypoints")
    if not isinstance(ep, dict):
        errors.append("entrypoints must be a dict")
    else:
        for g, want in (("single_spec", want_single_ep), ("batch_specs", want_batch_ep)):
            if g not in ep:
                errors.append(f"entrypoints missing {g}")
                continue
            eg = ep[g]
            if not isinstance(eg, dict):
                errors.append(f"entrypoints.{g} must be a dict")
                continue
            for ck, wval in want.items():
                if eg.get(ck) != wval:
                    errors.append(f"entrypoints.{g}.{ck} must be {wval!r}")

    if errors:
        return {"ok": False, "overall_outcome": "error", "message": bad, "errors": errors}
    return {"ok": True, "overall_outcome": "success", "message": good, "errors": []}


def resolve_team_named_mission_api_entrypoint(scope: str, operation: str) -> Dict[str, Any]:
    """Resolve manifest-declared entrypoint name for scope and operation (pure, no ROS)."""
    sc = str(scope).strip()
    op = str(operation).strip()

    manifest = get_team_named_mission_api_manifest()
    val = validate_team_named_mission_api_manifest(manifest)
    if not val.get("ok"):
        return {
            "ok": False,
            "overall_outcome": "error",
            "scope": sc,
            "operation": op,
            "entrypoint": None,
            "summary": {"message": "", "error": "manifest validation failed"},
        }

    if sc not in ("single_spec", "batch_specs"):
        return {
            "ok": False,
            "overall_outcome": "error",
            "scope": sc,
            "operation": op,
            "entrypoint": None,
            "summary": {"message": "", "error": "unsupported scope"},
        }

    if op not in ("inspect", "inspect_checked", "execute", "execute_checked"):
        return {
            "ok": False,
            "overall_outcome": "error",
            "scope": sc,
            "operation": op,
            "entrypoint": None,
            "summary": {"message": "", "error": "unsupported operation"},
        }

    ep = manifest["entrypoints"][sc][op]
    return {
        "ok": True,
        "overall_outcome": "success",
        "scope": sc,
        "operation": op,
        "entrypoint": ep,
        "summary": {"message": "Mission API entrypoint resolved successfully.", "error": None},
    }


def validate_team_named_mission_api_request(request: Any) -> Dict[str, Any]:
    """Validate a proposed Layer B mission API call (pure, no ROS)."""
    manifest = get_team_named_mission_api_manifest()
    mval = validate_team_named_mission_api_manifest(manifest)

    sc = ""
    op = ""
    if isinstance(request, dict):
        rs = request.get("scope")
        ro = request.get("operation")
        sc = "" if rs is None else str(rs).strip()
        op = "" if ro is None else str(ro).strip()

    if not mval.get("ok"):
        return {
            "ok": False,
            "overall_outcome": "error",
            "scope": sc,
            "operation": op,
            "entrypoint": None,
            "summary": {"message": "", "error": "manifest validation failed"},
        }

    if not isinstance(request, dict):
        return {
            "ok": False,
            "overall_outcome": "error",
            "scope": "",
            "operation": "",
            "entrypoint": None,
            "summary": {"message": "", "error": "request must be a dict"},
        }

    if sc == "":
        return {
            "ok": False,
            "overall_outcome": "error",
            "scope": sc,
            "operation": op,
            "entrypoint": None,
            "summary": {"message": "", "error": "missing scope"},
        }

    if op == "":
        return {
            "ok": False,
            "overall_outcome": "error",
            "scope": sc,
            "operation": op,
            "entrypoint": None,
            "summary": {"message": "", "error": "missing operation"},
        }

    r = resolve_team_named_mission_api_entrypoint(sc, op)
    if not r.get("ok"):
        return r

    return {
        "ok": True,
        "overall_outcome": "success",
        "scope": sc,
        "operation": op,
        "entrypoint": r.get("entrypoint"),
        "summary": {"message": "Mission API request validated successfully.", "error": None},
    }


def plan_team_named_mission_api_call(request: Any) -> Dict[str, Any]:
    """Convert a validated API request into a compact execution-free plan (pure, no ROS)."""
    v = validate_team_named_mission_api_request(request)
    sc = str(v.get("scope", ""))
    op = str(v.get("operation", ""))
    vsum = v.get("summary") if isinstance(v.get("summary"), dict) else {}
    verr = vsum.get("error")
    err_out = None if verr is None else str(verr)

    if not v.get("ok"):
        return {
            "ok": False,
            "overall_outcome": "error",
            "scope": sc,
            "operation": op,
            "entrypoint": None,
            "execution_kind": "invalid",
            "checked": None,
            "summary": {"message": "", "error": err_out},
        }

    opn = str(v.get("operation", ""))
    checked = opn in ("inspect_checked", "execute_checked")
    execution_kind = "offline" if opn in ("inspect", "inspect_checked") else "live"

    return {
        "ok": True,
        "overall_outcome": "success",
        "scope": sc,
        "operation": opn,
        "entrypoint": v.get("entrypoint"),
        "execution_kind": execution_kind,
        "checked": checked,
        "summary": {"message": "Mission API call plan generated successfully.", "error": None},
    }
