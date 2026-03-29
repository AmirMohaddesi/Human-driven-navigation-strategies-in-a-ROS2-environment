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
