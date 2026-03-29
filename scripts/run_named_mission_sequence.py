#!/usr/bin/env python3
"""
Sequential multi-robot named navigation using mission-agent + wait_for_goal.

Requires ROS + workspace sourced so `ros2 run multi_robot_mission_stack mission-agent` works.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def _helper_log(event: str, **fields: Any) -> None:
    line = json.dumps(
        {"component": "mission_sequence", "event": event, **fields},
        separators=(",", ":"),
        default=str,
    )
    sys.stderr.write(line + "\n")


def parse_step(raw: str) -> Tuple[str, str]:
    s = raw.strip()
    if ":" not in s:
        raise ValueError(f"invalid step {raw!r}; expected robot_id:location_name")
    rid, loc = s.split(":", 1)
    rid, loc = rid.strip(), loc.strip()
    if not rid or not loc:
        raise ValueError(f"invalid step {raw!r}; empty robot_id or location_name")
    return rid, loc


def script_dir() -> Path:
    return Path(__file__).resolve().parent


def parse_last_json_line(text: str) -> Optional[Dict[str, Any]]:
    if not (text or "").strip():
        return None
    for line in reversed(text.splitlines()):
        s = line.strip()
        if len(s) < 2 or not s.startswith("{") or not s.endswith("}"):
            continue
        try:
            return json.loads(s)
        except json.JSONDecodeError:
            continue
    return None


def parse_agent_json(stdout: str) -> Dict[str, Any]:
    found = parse_last_json_line(stdout)
    if found is None:
        raise ValueError("no JSON object in stdout")
    return found


def run_navigate(robot_id: str, location_name: str, call_timeout: float) -> Tuple[int, str, str]:
    cmd = [
        "ros2",
        "run",
        "multi_robot_mission_stack",
        "mission-agent",
        "--ros",
        "navigate",
        "--robot-id",
        robot_id,
        "--location-name",
        location_name,
    ]
    proc = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=call_timeout,
        check=False,
    )
    return proc.returncode, proc.stdout or "", proc.stderr or ""


def run_wait_for_goal(
    robot_id: str,
    goal_id: str,
    per_goal_timeout: float,
    poll_interval: float,
    call_timeout: float,
) -> Tuple[int, str, str]:
    wfg = script_dir() / "wait_for_goal.py"
    cmd = [
        sys.executable,
        str(wfg),
        "--robot-id",
        robot_id,
        "--goal-id",
        goal_id,
        "--timeout",
        str(per_goal_timeout),
        "--poll-interval",
        str(poll_interval),
        "--call-timeout",
        str(call_timeout),
    ]
    proc = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=per_goal_timeout + call_timeout + 30.0,
        check=False,
    )
    out = (proc.stdout or "").strip()
    err = (proc.stderr or "").strip()
    return proc.returncode, out, err


def step_succeeded(wait_exit: int, wait_stdout: str) -> Tuple[bool, Optional[Dict[str, Any]]]:
    payload = parse_last_json_line(wait_stdout)
    if wait_exit != 0:
        return False, payload
    if payload is None:
        return False, None
    oc = str(payload.get("outcome", "")).lower()
    return oc == "succeeded", payload


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run sequential named-navigation steps (robot_id:location_name)."
    )
    parser.add_argument(
        "--steps",
        nargs="+",
        required=True,
        metavar="ROBOT:LOCATION",
        help="Steps like robot1:base robot2:test_goal",
    )
    parser.add_argument("--per-goal-timeout", type=float, default=120.0)
    parser.add_argument("--poll-interval", type=float, default=1.0)
    parser.add_argument(
        "--continue-on-failure",
        action="store_true",
        help="Run remaining steps after a failure (still exit 1 if any failed).",
    )
    parser.add_argument(
        "--nav-call-timeout",
        type=float,
        default=60.0,
        help="Max seconds for each ros2 navigate invocation.",
    )
    parser.add_argument(
        "--wait-call-timeout",
        type=float,
        default=25.0,
        help="Per-query timeout passed through to wait_for_goal.py --call-timeout.",
    )
    args = parser.parse_args()

    steps_parsed: List[Tuple[str, str]] = []
    try:
        for raw in args.steps:
            steps_parsed.append(parse_step(raw))
    except ValueError as exc:
        _helper_log("sequence_parse_error", error=str(exc))
        summary = {
            "overall_outcome": "failure",
            "total_steps": len(args.steps),
            "steps_run": 0,
            "succeeded_count": 0,
            "failed_count": 0,
            "stopped_early": True,
            "error": str(exc),
            "steps": [],
        }
        print(json.dumps(summary, separators=(",", ":")))
        return 1

    _helper_log(
        "sequence_start",
        total_steps=len(steps_parsed),
        continue_on_failure=bool(args.continue_on_failure),
    )

    results: List[Dict[str, Any]] = []
    succeeded = 0
    failed = 0
    stopped_early = False

    for idx, (robot_id, location_name) in enumerate(steps_parsed):
        rec: Dict[str, Any] = {
            "index": idx,
            "robot_id": robot_id,
            "location_name": location_name,
            "goal_id": None,
            "nav_exit": None,
            "wait_exit": None,
            "wait_outcome": None,
            "wait_detail": None,
            "step_outcome": "failed",
            "error": None,
        }
        _helper_log(
            "step_start",
            step_index=idx,
            robot_id=robot_id,
            location_name=location_name,
        )
        try:
            ncode, nout, nerr = run_navigate(
                robot_id, location_name, args.nav_call_timeout
            )
            rec["nav_exit"] = ncode
            if ncode != 0:
                rec["error"] = "navigate_command_failed"
                rec["stderr"] = (nerr or "")[:400]
                nav_fail_json = parse_last_json_line(nout)
                if nav_fail_json is not None:
                    rec["nav_response"] = nav_fail_json
                failed += 1
                _helper_log(
                    "step_failed",
                    step_index=idx,
                    robot_id=robot_id,
                    location_name=location_name,
                    phase="navigate",
                    error=rec.get("error"),
                    nav_exit=ncode,
                )
                results.append(rec)
                if not args.continue_on_failure:
                    stopped_early = True
                    break
                continue
            try:
                nav_json = parse_agent_json(nout)
            except (json.JSONDecodeError, ValueError):
                rec["error"] = "navigate_json_parse_failed"
                rec["stdout"] = (nout or "")[:400]
                failed += 1
                _helper_log(
                    "step_failed",
                    step_index=idx,
                    robot_id=robot_id,
                    location_name=location_name,
                    phase="navigate",
                    error=rec.get("error"),
                )
                results.append(rec)
                if not args.continue_on_failure:
                    stopped_early = True
                    break
                continue

            st = str(nav_json.get("status", "")).lower()
            ns = str(nav_json.get("nav_status", "")).lower()
            if st in ("failed", "failure") or ns == "rejected":
                rec["error"] = "navigate_rejected_or_failed"
                rec["nav_response"] = nav_json
                failed += 1
                _helper_log(
                    "step_failed",
                    step_index=idx,
                    robot_id=robot_id,
                    location_name=location_name,
                    phase="navigate",
                    error=rec.get("error"),
                )
                results.append(rec)
                if not args.continue_on_failure:
                    stopped_early = True
                    break
                continue

            gid = nav_json.get("goal_id")
            if not gid:
                rec["error"] = "navigate_missing_goal_id"
                rec["nav_response"] = nav_json
                failed += 1
                _helper_log(
                    "step_failed",
                    step_index=idx,
                    robot_id=robot_id,
                    location_name=location_name,
                    phase="navigate",
                    error=rec.get("error"),
                )
                results.append(rec)
                if not args.continue_on_failure:
                    stopped_early = True
                    break
                continue

            rec["goal_id"] = str(gid)
            _helper_log(
                "navigate_ok",
                step_index=idx,
                robot_id=robot_id,
                goal_id=str(gid),
                nav_exit=ncode,
            )
            wcode, wout, werr = run_wait_for_goal(
                robot_id,
                str(gid),
                args.per_goal_timeout,
                args.poll_interval,
                args.wait_call_timeout,
            )
            rec["wait_exit"] = wcode
            ok, detail = step_succeeded(wcode, wout)
            if detail is not None:
                rec["wait_detail"] = detail
                rec["wait_outcome"] = detail.get("outcome")
            if werr:
                rec["wait_stderr"] = werr[:400]
            if ok:
                rec["step_outcome"] = "succeeded"
                succeeded += 1
                _helper_log(
                    "step_complete",
                    step_index=idx,
                    robot_id=robot_id,
                    goal_id=str(gid),
                    step_outcome="succeeded",
                    wait_exit=wcode,
                    wait_outcome=rec.get("wait_outcome"),
                )
            else:
                rec["error"] = "wait_terminal_not_succeeded"
                if wout:
                    rec["wait_stdout"] = wout[:500]
                failed += 1
                _helper_log(
                    "step_failed",
                    step_index=idx,
                    robot_id=robot_id,
                    goal_id=str(gid),
                    phase="wait",
                    error=rec.get("error"),
                    wait_exit=wcode,
                    wait_outcome=rec.get("wait_outcome"),
                )
                results.append(rec)
                if not args.continue_on_failure:
                    stopped_early = True
                    break
                continue

            results.append(rec)
        except subprocess.TimeoutExpired:
            rec["error"] = "subprocess_timeout"
            failed += 1
            _helper_log(
                "step_failed",
                step_index=idx,
                robot_id=robot_id,
                location_name=location_name,
                phase="subprocess",
                error=rec.get("error"),
            )
            results.append(rec)
            if not args.continue_on_failure:
                stopped_early = True
                break
        except Exception as exc:
            rec["error"] = f"internal_error:{exc}"
            failed += 1
            _helper_log(
                "step_failed",
                step_index=idx,
                robot_id=robot_id,
                location_name=location_name,
                phase="internal",
                error=rec.get("error"),
            )
            results.append(rec)
            if not args.continue_on_failure:
                stopped_early = True
                break

    total = len(steps_parsed)
    steps_run = len(results)
    overall = "success" if failed == 0 and steps_run == total else "failure"
    summary = {
        "overall_outcome": overall,
        "total_steps": total,
        "steps_run": steps_run,
        "succeeded_count": succeeded,
        "failed_count": failed,
        "stopped_early": stopped_early,
        "continue_on_failure": bool(args.continue_on_failure),
        "steps": results,
    }
    _helper_log(
        "sequence_finished",
        overall_outcome=overall,
        total_steps=total,
        steps_run=steps_run,
        succeeded_count=succeeded,
        failed_count=failed,
        stopped_early=stopped_early,
    )
    print(json.dumps(summary, separators=(",", ":")))
    return 0 if overall == "success" else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BrokenPipeError:
        raise SystemExit(1)
