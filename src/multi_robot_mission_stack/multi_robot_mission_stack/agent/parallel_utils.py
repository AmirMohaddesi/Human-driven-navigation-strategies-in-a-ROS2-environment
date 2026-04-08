"""In-process parallel named navigation: concurrent submit, then concurrent wait."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Any, Dict, List, Optional, TYPE_CHECKING

from .navigate_failure_classification_v51 import navigate_failure_kind
from .sequence_utils import StepInput, _navigate_named_command, _validate_steps
from .wait_utils import wait_for_terminal_navigation_state

if TYPE_CHECKING:
    from .mission_agent_facade import MissionAgentFacade


def duplicate_robot_error(steps: List[StepInput]) -> Optional[str]:
    seen: set[str] = set()
    for s in steps:
        rid = str(s["robot_id"]).strip()
        if rid in seen:
            return f"duplicate robot_id {rid!r} (one active goal per robot)"
        seen.add(rid)
    return None


def _submit_ok(res: Dict[str, Any]) -> tuple[bool, str]:
    st = str(res.get("status", "")).lower()
    ns = str(res.get("nav_status", "")).lower()
    if st in ("failed", "failure") or ns == "rejected":
        return False, "navigate_rejected_or_failed"
    gid = res.get("goal_id")
    if not gid or not str(gid).strip():
        return False, "missing_goal_id"
    return True, ""


def _failure_summary(
    *,
    err: str,
    total_steps: int,
) -> Dict[str, Any]:
    return {
        "overall_outcome": "failure",
        "total_steps": total_steps,
        "submitted_count": 0,
        "succeeded_count": 0,
        "failed_count": 0,
        "steps": [],
        "error": err,
    }


def _empty_step_rec(index: int, robot_id: str, location_name: str) -> Dict[str, Any]:
    return {
        "index": index,
        "robot_id": robot_id,
        "location_name": location_name,
        "navigate_result": {},
        "goal_id": "",
        "wait_result": None,
        "step_outcome": "failed",
        "error": None,
    }


def _submit_worker(
    facade: MissionAgentFacade,
    index: int,
    robot_id: str,
    location_name: str,
) -> Dict[str, Any]:
    rid = str(robot_id).strip()
    loc = str(location_name).strip()
    rec = _empty_step_rec(index, rid, loc)
    try:
        nav = facade.handle_command(_navigate_named_command(rid, loc))
    except Exception as exc:
        rec["navigate_result"] = {"status": "failed", "message": str(exc)}
        rec["error"] = "navigate_exception"
        return rec

    if not isinstance(nav, dict):
        rec["navigate_result"] = {"status": "failed", "message": "non_dict_navigate_result"}
        rec["error"] = "invalid_navigate_result"
        return rec

    rec["navigate_result"] = nav
    ok, reason = _submit_ok(nav)
    if not ok:
        rec["error"] = reason
        k = navigate_failure_kind(nav)
        if k is not None:
            rec["navigate_failure_kind"] = k
        return rec

    rec["goal_id"] = str(nav["goal_id"]).strip()
    return rec


def _wait_worker(
    facade: MissionAgentFacade,
    rec: Dict[str, Any],
    per_goal_timeout_sec: float,
    poll_interval_sec: float,
) -> Dict[str, Any]:
    if not rec.get("goal_id"):
        rec["wait_result"] = None
        rec["error"] = rec.get("error") or "wait_skipped_no_goal_id"
        return rec

    rid = str(rec["robot_id"])
    gid = str(rec["goal_id"])
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
        return rec

    rec["wait_result"] = wait_res
    woc = str(wait_res.get("outcome", "")).lower()
    if woc == "succeeded":
        rec["step_outcome"] = "succeeded"
        rec["error"] = None
    else:
        rec["error"] = f"wait_outcome_{woc}" if woc else "wait_outcome_unknown"
    return rec


def run_parallel_named_navigation(
    facade: MissionAgentFacade,
    steps: List[StepInput],
    *,
    per_goal_timeout_sec: float,
    poll_interval_sec: float,
) -> Dict[str, Any]:
    """
    Submit all named navigations concurrently, then wait on all goal_ids concurrently.
    One goal per robot; duplicate ``robot_id`` values are rejected.
    """
    err = _validate_steps(steps)
    if err is not None:
        n = len(steps) if isinstance(steps, list) else 0
        out = _failure_summary(err=err, total_steps=n)
        return out

    total = len(steps)
    dup = duplicate_robot_error(steps)
    if dup is not None:
        out = _failure_summary(err=dup, total_steps=total)
        return out

    n_workers = max(1, total)
    records: List[Optional[Dict[str, Any]]] = [None] * total

    with ThreadPoolExecutor(max_workers=n_workers) as ex:
        futs = {
            ex.submit(
                _submit_worker,
                facade,
                i,
                str(steps[i]["robot_id"]).strip(),
                str(steps[i]["location_name"]).strip(),
            ): i
            for i in range(total)
        }
        for fut in as_completed(futs):
            idx = futs[fut]
            records[idx] = fut.result()

    assert all(r is not None for r in records)
    rec_list: List[Dict[str, Any]] = [r for r in records if r is not None]

    submitted_count = sum(1 for r in rec_list if r.get("goal_id"))

    with ThreadPoolExecutor(max_workers=n_workers) as ex:
        futs = {
            ex.submit(
                _wait_worker,
                facade,
                rec,
                per_goal_timeout_sec,
                poll_interval_sec,
            ): int(rec["index"])
            for rec in rec_list
        }
        for fut in as_completed(futs):
            idx = futs[fut]
            rec_list[idx] = fut.result()

    final_steps = sorted(rec_list, key=lambda x: int(x["index"]))
    succeeded_count = sum(1 for s in final_steps if s.get("step_outcome") == "succeeded")
    failed_count = total - succeeded_count
    overall = "success" if succeeded_count == total else "failure"

    return {
        "overall_outcome": overall,
        "total_steps": total,
        "submitted_count": submitted_count,
        "succeeded_count": succeeded_count,
        "failed_count": failed_count,
        "steps": final_steps,
    }
