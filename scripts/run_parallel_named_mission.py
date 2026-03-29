#!/usr/bin/env python3
"""
Parallel submission + parallel wait for multi-robot named navigation (validation helper).

Submits all navigate calls (concurrently), then waits on all goal_ids (concurrently)
via scripts/wait_for_goal.py. Not a production orchestrator.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

_rms: Any = None


def _load_rms() -> Any:
    global _rms
    if _rms is not None:
        return _rms
    path = Path(__file__).resolve().parent / "run_named_mission_sequence.py"
    spec = importlib.util.spec_from_file_location("_rms_parallel", path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    _rms = mod
    return _rms


def _plog(event: str, **fields: Any) -> None:
    sys.stderr.write(
        json.dumps(
            {"component": "parallel_named_mission", "event": event, **fields},
            separators=(",", ":"),
            default=str,
        )
        + "\n"
    )


def duplicate_robot_error(steps: List[Tuple[str, str]]) -> Optional[str]:
    seen: set[str] = set()
    for rid, _ in steps:
        if rid in seen:
            return f"duplicate robot_id {rid!r} (one active goal per robot)"
        seen.add(rid)
    return None


def empty_rec(index: int, robot_id: str, location_name: str) -> Dict[str, Any]:
    return {
        "index": index,
        "robot_id": robot_id,
        "location_name": location_name,
        "nav_exit": None,
        "nav_response": None,
        "goal_id": None,
        "wait_exit": None,
        "wait_detail": None,
        "step_outcome": "failed",
        "error": None,
    }


def finalize_step_outcome(rec: Dict[str, Any]) -> None:
    """succeeded only if navigate ok, goal_id, wait 0, outcome succeeded."""
    if rec.get("nav_exit") != 0:
        rec["step_outcome"] = "failed"
        if not rec.get("error"):
            rec["error"] = "navigate_command_failed"
        return
    if not rec.get("goal_id"):
        rec["step_outcome"] = "failed"
        if not rec.get("error"):
            rec["error"] = "missing_goal_id"
        return
    if rec.get("wait_exit") is None:
        rec["step_outcome"] = "failed"
        if not rec.get("error"):
            rec["error"] = "wait_skipped"
        return
    if rec.get("wait_exit") != 0:
        rec["step_outcome"] = "failed"
        if not rec.get("error"):
            rec["error"] = "wait_failed"
        return
    detail = rec.get("wait_detail")
    if not isinstance(detail, dict):
        rec["step_outcome"] = "failed"
        if not rec.get("error"):
            rec["error"] = "wait_missing_detail"
        return
    if str(detail.get("outcome", "")).lower() != "succeeded":
        rec["step_outcome"] = "failed"
        if not rec.get("error"):
            rec["error"] = "wait_outcome_not_succeeded"
        return
    rec["step_outcome"] = "succeeded"
    rec["error"] = None


def submit_worker(
    index: int,
    robot_id: str,
    location_name: str,
    nav_timeout: float,
) -> Dict[str, Any]:
    rms = _load_rms()
    rec = empty_rec(index, robot_id, location_name)
    _plog(
        "submit_start",
        step_index=index,
        robot_id=robot_id,
        location_name=location_name,
    )
    try:
        ncode, nout, nerr = rms.run_navigate(robot_id, location_name, nav_timeout)
    except subprocess.TimeoutExpired:
        rec["nav_exit"] = -1
        rec["error"] = "navigate_subprocess_timeout"
        _plog("submit_done", step_index=index, robot_id=robot_id, ok=False)
        return rec

    rec["nav_exit"] = ncode
    nav_json = rms.parse_last_json_line(nout)
    if nav_json is not None:
        rec["nav_response"] = nav_json

    if ncode != 0:
        rec["error"] = "navigate_command_failed"
        if nerr:
            rec["stderr"] = nerr[:400]
        _plog("submit_done", step_index=index, robot_id=robot_id, ok=False)
        return rec

    try:
        nav_json = rms.parse_agent_json(nout)
        rec["nav_response"] = nav_json
    except (json.JSONDecodeError, ValueError):
        rec["error"] = "navigate_json_parse_failed"
        _plog("submit_done", step_index=index, robot_id=robot_id, ok=False)
        return rec

    st = str(nav_json.get("status", "")).lower()
    ns = str(nav_json.get("nav_status", "")).lower()
    if st in ("failed", "failure") or ns == "rejected":
        rec["error"] = "navigate_rejected_or_failed"
        _plog("submit_done", step_index=index, robot_id=robot_id, ok=False)
        return rec

    gid = nav_json.get("goal_id")
    if not gid:
        rec["error"] = "navigate_missing_goal_id"
        _plog("submit_done", step_index=index, robot_id=robot_id, ok=False)
        return rec

    rec["goal_id"] = str(gid)
    _plog(
        "submit_done",
        step_index=index,
        robot_id=robot_id,
        goal_id=rec["goal_id"],
        ok=True,
    )
    return rec


def wait_worker(
    rec: Dict[str, Any],
    per_goal_timeout: float,
    poll_interval: float,
    wait_call_timeout: float,
) -> Dict[str, Any]:
    rms = _load_rms()
    idx = rec["index"]
    robot_id = rec["robot_id"]
    if not rec.get("goal_id"):
        rec["error"] = rec.get("error") or "wait_skipped_no_goal_id"
        rec["wait_exit"] = None
        _plog("wait_skipped", step_index=idx, robot_id=robot_id, reason=rec["error"])
        return rec

    _plog(
        "wait_start",
        step_index=idx,
        robot_id=robot_id,
        goal_id=rec["goal_id"],
    )
    try:
        wcode, wout, werr = rms.run_wait_for_goal(
            robot_id,
            str(rec["goal_id"]),
            per_goal_timeout,
            poll_interval,
            wait_call_timeout,
        )
    except subprocess.TimeoutExpired:
        rec["wait_exit"] = -1
        rec["error"] = "wait_subprocess_timeout"
        _plog("wait_done", step_index=idx, robot_id=robot_id, ok=False)
        return rec

    rec["wait_exit"] = wcode
    ok, detail = rms.step_succeeded(wcode, wout)
    if detail is not None:
        rec["wait_detail"] = detail
    if werr:
        rec["wait_stderr"] = werr[:400]
    if not ok and not rec.get("error"):
        rec["error"] = "wait_terminal_not_succeeded"

    _plog(
        "wait_done",
        step_index=idx,
        robot_id=robot_id,
        goal_id=rec.get("goal_id"),
        wait_exit=wcode,
        ok=ok,
    )
    return rec


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Parallel navigate submit + parallel wait (validation helper)."
    )
    parser.add_argument(
        "--steps",
        nargs="+",
        required=True,
        metavar="ROBOT:LOCATION",
    )
    parser.add_argument("--per-goal-timeout", type=float, default=120.0)
    parser.add_argument("--poll-interval", type=float, default=1.0)
    parser.add_argument("--nav-call-timeout", type=float, default=60.0)
    parser.add_argument("--wait-call-timeout", type=float, default=25.0)
    parser.add_argument(
        "--max-workers",
        type=int,
        default=0,
        help="Thread pool size (default: number of steps).",
    )
    args = parser.parse_args()
    rms = _load_rms()

    try:
        steps_parsed = [rms.parse_step(s) for s in args.steps]
    except ValueError as exc:
        summary = {
            "overall_outcome": "failure",
            "total_steps": len(args.steps),
            "submitted_count": 0,
            "succeeded_count": 0,
            "failed_count": 0,
            "error": str(exc),
            "steps": [],
        }
        print(json.dumps(summary, separators=(",", ":")))
        return 1

    dup = duplicate_robot_error(steps_parsed)
    if dup:
        summary = {
            "overall_outcome": "failure",
            "total_steps": len(steps_parsed),
            "submitted_count": 0,
            "succeeded_count": 0,
            "failed_count": 0,
            "error": dup,
            "steps": [],
        }
        print(json.dumps(summary, separators=(",", ":")))
        return 1

    total = len(steps_parsed)
    workers = args.max_workers if args.max_workers > 0 else total
    workers = max(1, min(workers, total))

    _plog("parallel_start", total_steps=total, workers=workers)

    records: List[Optional[Dict[str, Any]]] = [None] * total
    with ThreadPoolExecutor(max_workers=workers) as ex:
        futs = {
            ex.submit(
                submit_worker,
                i,
                rid,
                loc,
                args.nav_call_timeout,
            ): i
            for i, (rid, loc) in enumerate(steps_parsed)
        }
        for fut in as_completed(futs):
            idx = futs[fut]
            records[idx] = fut.result()

    assert all(r is not None for r in records)
    rec_list: List[Dict[str, Any]] = [r for r in records if r is not None]

    with ThreadPoolExecutor(max_workers=workers) as ex:
        futs = {
            ex.submit(
                wait_worker,
                rec,
                args.per_goal_timeout,
                args.poll_interval,
                args.wait_call_timeout,
            ): int(rec["index"])
            for rec in rec_list
        }
        for fut in as_completed(futs):
            idx = futs[fut]
            rec_list[idx] = fut.result()

    final_steps = sorted(rec_list, key=lambda x: int(x["index"]))
    for rec in final_steps:
        finalize_step_outcome(rec)

    submitted_count = sum(1 for s in final_steps if s.get("goal_id"))
    succeeded_count = sum(1 for s in final_steps if s.get("step_outcome") == "succeeded")
    failed_count = total - succeeded_count
    overall = "success" if succeeded_count == total else "failure"

    summary = {
        "overall_outcome": overall,
        "total_steps": total,
        "submitted_count": submitted_count,
        "succeeded_count": succeeded_count,
        "failed_count": failed_count,
        "steps": sorted(final_steps, key=lambda x: x["index"]),
    }
    _plog(
        "parallel_finished",
        overall_outcome=overall,
        submitted_count=submitted_count,
        succeeded_count=succeeded_count,
        failed_count=failed_count,
    )
    print(json.dumps(summary, separators=(",", ":")))
    return 0 if overall == "success" else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BrokenPipeError:
        raise SystemExit(1)
