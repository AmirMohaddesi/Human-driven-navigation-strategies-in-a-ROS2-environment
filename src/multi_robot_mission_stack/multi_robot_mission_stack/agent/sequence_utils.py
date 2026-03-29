"""In-process sequential named navigation using MissionAgentFacade + wait_utils."""

from __future__ import annotations

from typing import Any, Dict, List, Optional, TYPE_CHECKING

from .wait_utils import wait_for_terminal_navigation_state

if TYPE_CHECKING:
    from .mission_agent_facade import MissionAgentFacade

StepInput = Dict[str, str]


def _navigate_named_command(robot_id: str, location_name: str) -> Dict[str, Any]:
    return {
        "type": "navigate",
        "target": "named_location",
        "robot_id": str(robot_id).strip(),
        "location_name": str(location_name).strip(),
    }


def _navigate_ok(res: Dict[str, Any]) -> tuple[bool, str]:
    st = str(res.get("status", "")).lower()
    if st in ("failed", "failure"):
        return False, "navigate_status_failed"
    gid = res.get("goal_id")
    if not gid or not str(gid).strip():
        return False, "missing_goal_id"
    return True, ""


def _validate_steps(steps: Any) -> Optional[str]:
    if not isinstance(steps, list):
        return "steps must be a list"
    for i, raw in enumerate(steps):
        if not isinstance(raw, dict):
            return f"step {i} must be a dict"
        rid = raw.get("robot_id")
        loc = raw.get("location_name")
        if not isinstance(rid, str) or not rid.strip():
            return f"step {i} missing non-empty robot_id"
        if not isinstance(loc, str) or not loc.strip():
            return f"step {i} missing non-empty location_name"
    return None


def run_sequential_named_navigation(
    facade: MissionAgentFacade,
    steps: List[StepInput],
    *,
    per_goal_timeout_sec: float,
    poll_interval_sec: float,
    continue_on_failure: bool = False,
) -> Dict[str, Any]:
    """
    For each step: named navigate via ``handle_command``, then
    ``wait_for_terminal_navigation_state`` until terminal or timeout.

    ``step_outcome`` is ``succeeded`` only when navigate yields a non-empty ``goal_id``
    and wait ``outcome`` is ``succeeded``.
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
    stopped_early = False

    for idx, step in enumerate(steps):
        rid = str(step["robot_id"]).strip()
        loc = str(step["location_name"]).strip()
        rec: Dict[str, Any] = {
            "index": idx,
            "robot_id": rid,
            "location_name": loc,
            "navigate_result": {},
            "goal_id": "",
            "wait_result": None,
            "step_outcome": "failed",
            "error": None,
        }

        try:
            nav = facade.handle_command(_navigate_named_command(rid, loc))
        except Exception as exc:
            rec["navigate_result"] = {"status": "failed", "message": str(exc)}
            rec["error"] = "navigate_exception"
            failed_count += 1
            out_steps.append(rec)
            if not continue_on_failure:
                stopped_early = idx < total - 1
                break
            continue

        if not isinstance(nav, dict):
            rec["navigate_result"] = {"status": "failed", "message": "non_dict_navigate_result"}
            rec["error"] = "invalid_navigate_result"
            failed_count += 1
            out_steps.append(rec)
            if not continue_on_failure:
                stopped_early = idx < total - 1
                break
            continue

        rec["navigate_result"] = nav
        ok_nav, reason = _navigate_ok(nav)
        if not ok_nav:
            rec["error"] = reason
            failed_count += 1
            out_steps.append(rec)
            if not continue_on_failure:
                stopped_early = idx < total - 1
                break
            continue

        gid = str(nav["goal_id"]).strip()
        rec["goal_id"] = gid

        try:
            wait_res = wait_for_terminal_navigation_state(
                facade,
                rid,
                gid,
                timeout_sec=float(per_goal_timeout_sec),
                poll_interval_sec=float(poll_interval_sec),
            )
        except Exception as exc:
            rec["wait_result"] = {
                "outcome": "failed",
                "message": str(exc),
                "robot_id": rid,
                "goal_id": gid,
            }
            rec["error"] = "wait_exception"
            failed_count += 1
            out_steps.append(rec)
            if not continue_on_failure:
                stopped_early = idx < total - 1
                break
            continue

        rec["wait_result"] = wait_res
        woc = str(wait_res.get("outcome", "")).lower()
        if woc == "succeeded":
            rec["step_outcome"] = "succeeded"
            rec["error"] = None
            succeeded_count += 1
        else:
            rec["error"] = f"wait_outcome_{woc}" if woc else "wait_outcome_unknown"
            failed_count += 1

        out_steps.append(rec)

        if rec["step_outcome"] != "succeeded" and not continue_on_failure:
            stopped_early = idx < total - 1
            break

    steps_run = len(out_steps)
    overall = "success" if failed_count == 0 and steps_run == total else "failure"

    return {
        "overall_outcome": overall,
        "total_steps": total,
        "steps_run": steps_run,
        "succeeded_count": succeeded_count,
        "failed_count": failed_count,
        "stopped_early": stopped_early,
        "continue_on_failure": bool(continue_on_failure),
        "steps": out_steps,
    }
