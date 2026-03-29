#!/usr/bin/env python3
"""
Stress / validation driver for helper-level parallel multi-robot missions.

Invokes scripts/run_parallel_named_mission.py and optional pose+cancel overlap.
All operator aggregates on stdout as one JSON object; progress on stderr.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

_REPO_ROOT = Path(__file__).resolve().parents[1]
_PARALLEL = _REPO_ROOT / "scripts" / "run_parallel_named_mission.py"
_WAIT = _REPO_ROOT / "scripts" / "wait_for_goal.py"
_RMS: Any = None


def _load_rms() -> Any:
    global _RMS
    if _RMS is not None:
        return _RMS
    p = _REPO_ROOT / "scripts" / "run_named_mission_sequence.py"
    spec = importlib.util.spec_from_file_location("_rms_stress", p)
    assert spec and spec.loader
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    _RMS = m
    return _RMS


def _slog(event: str, **fields: Any) -> None:
    sys.stderr.write(
        json.dumps(
            {"component": "stress_parallel", "event": event, **fields},
            separators=(",", ":"),
            default=str,
        )
        + "\n"
    )


def isolation_check(
    summary: Dict[str, Any],
    expected: List[Tuple[str, str]],
) -> List[str]:
    """
    Verify robot_id/location per index and unique goal_ids when both present.
    expected: list of (robot_id, location_name) in step order.
    """
    issues: List[str] = []
    steps = summary.get("steps")
    if not isinstance(steps, list):
        return ["missing_or_invalid_steps_array"]
    ordered = sorted(steps, key=lambda s: int(s.get("index", -1)))
    if len(ordered) != len(expected):
        issues.append(f"step_count:{len(ordered)}_expected_{len(expected)}")
    for i, exp in enumerate(expected):
        if i >= len(ordered):
            issues.append(f"missing_step_index_{i}")
            continue
        s = ordered[i]
        if s.get("robot_id") != exp[0]:
            issues.append(f"index{i}_robot_id:{s.get('robot_id')}!={exp[0]}")
        if s.get("location_name") != exp[1]:
            issues.append(f"index{i}_location:{s.get('location_name')}!={exp[1]}")
    gids = [s.get("goal_id") for s in ordered if s.get("goal_id")]
    if len(gids) >= 2 and len(set(gids)) < len(gids):
        issues.append("duplicate_goal_id_across_steps")
    return issues


def count_timeouts_and_cancel(summary: Dict[str, Any]) -> Tuple[int, int]:
    timeouts = 0
    cancels = 0
    for s in summary.get("steps") or []:
        if not isinstance(s, dict):
            continue
        wd = s.get("wait_detail")
        if isinstance(wd, dict):
            oc = str(wd.get("outcome", "")).lower()
            if oc == "timeout":
                timeouts += 1
                continue
            if oc == "cancelled":
                cancels += 1
                continue
        err = str(s.get("error") or "")
        if "timeout" in err.lower():
            timeouts += 1
    return timeouts, cancels


def run_parallel(
    steps: List[str],
    per_goal_timeout: float,
    poll_interval: float,
    nav_timeout: float,
    wait_call_timeout: float,
) -> Tuple[int, Dict[str, Any]]:
    cmd = [
        sys.executable,
        str(_PARALLEL),
        "--steps",
        *steps,
        "--per-goal-timeout",
        str(per_goal_timeout),
        "--poll-interval",
        str(poll_interval),
        "--nav-call-timeout",
        str(nav_timeout),
        "--wait-call-timeout",
        str(wait_call_timeout),
    ]
    proc = subprocess.run(
        cmd,
        cwd=str(_REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=per_goal_timeout * len(steps) + nav_timeout * len(steps) + 120.0,
        check=False,
    )
    raw = (proc.stdout or "").strip()
    try:
        data = json.loads(raw.splitlines()[-1] if raw else "{}")
    except json.JSONDecodeError:
        data = {
            "overall_outcome": "failure",
            "parse_error": True,
            "stdout": raw[:800],
            "stderr": (proc.stderr or "")[:800],
            "steps": [],
        }
    return proc.returncode, data


def run_navigate_pose(robot_id: str, x: float, y: float, yaw: float, timeout: float) -> Tuple[int, Dict[str, Any]]:
    proc = subprocess.run(
        [
            "ros2",
            "run",
            "multi_robot_mission_stack",
            "mission-agent",
            "--ros",
            "navigate-pose",
            "--robot-id",
            robot_id,
            "--x",
            str(x),
            "--y",
            str(y),
            "--yaw",
            str(yaw),
        ],
        cwd=str(_REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )
    rms = _load_rms()
    j = rms.parse_last_json_line(proc.stdout or "")
    return proc.returncode, j if isinstance(j, dict) else {}


def run_cancel(robot_id: str, goal_id: str, timeout: float) -> Tuple[int, str]:
    proc = subprocess.run(
        [
            "ros2",
            "run",
            "multi_robot_mission_stack",
            "mission-agent",
            "--ros",
            "cancel",
            "--robot-id",
            robot_id,
            "--goal-id",
            goal_id,
        ],
        cwd=str(_REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )
    return proc.returncode, (proc.stdout or "").strip()


def run_wait_goal(
    robot_id: str, goal_id: str, per_goal_timeout: float, poll: float, call_to: float
) -> Tuple[int, Dict[str, Any]]:
    proc = subprocess.run(
        [
            sys.executable,
            str(_WAIT),
            "--robot-id",
            robot_id,
            "--goal-id",
            goal_id,
            "--timeout",
            str(per_goal_timeout),
            "--poll-interval",
            str(poll),
            "--call-timeout",
            str(call_to),
        ],
        cwd=str(_REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=per_goal_timeout + call_to + 60.0,
        check=False,
    )
    rms = _load_rms()
    line = (proc.stdout or "").strip().splitlines()
    last = line[-1] if line else ""
    try:
        return proc.returncode, json.loads(last) if last.startswith("{") else {}
    except json.JSONDecodeError:
        return proc.returncode, {}


def scenario_cancel_overlap(
    pose_x: float,
    pose_y: float,
    yaw: float,
    nav_to: float,
    wait_to: float,
    poll: float,
    call_to: float,
) -> Dict[str, Any]:
    """Long pose goals; cancel robot1 after brief delay; both should retain distinct outcomes."""
    _slog("cancel_overlap_start", robot1="robot1", robot2="robot2")

    def nav(rid: str) -> Tuple[str, str, Dict[str, Any]]:
        code, j = run_navigate_pose(rid, pose_x, pose_y, yaw, nav_to)
        gid = str(j.get("goal_id") or "") if j else ""
        return rid, gid, j

    with ThreadPoolExecutor(max_workers=2) as ex:
        f1 = ex.submit(nav, "robot1")
        f2 = ex.submit(nav, "robot2")
        r1, g1, j1 = f1.result()
        r2, g2, j2 = f2.result()

    time.sleep(0.4)
    cancel_ec = -1
    if g1:
        cancel_ec, _ = run_cancel("robot1", g1, 30.0)

    def wait_pair(rid: str, gid: str) -> Dict[str, Any]:
        if not gid:
            return {"robot_id": rid, "goal_id": None, "wait_exit": None, "wait_json": {}, "skipped": True}
        we, wj = run_wait_goal(rid, gid, wait_to, poll, call_to)
        return {"robot_id": rid, "goal_id": gid, "wait_exit": we, "wait_json": wj, "skipped": False}

    with ThreadPoolExecutor(max_workers=2) as ex:
        w1 = ex.submit(wait_pair, "robot1", g1)
        w2 = ex.submit(wait_pair, "robot2", g2)
        out1 = w1.result()
        out2 = w2.result()

    o1 = str(out1.get("wait_json", {}).get("outcome", "")).lower()
    o2 = str(out2.get("wait_json", {}).get("outcome", "")).lower()
    isolation = []
    if g1 and g2 and g1 == g2:
        isolation.append("same_goal_id_both_robots")
    healthy = (
        o1 == "cancelled"
        and o2 == "succeeded"
        and g1 != g2
        and bool(g1)
        and bool(g2)
    )
    result = {
        "scenario": "cancel_overlap_pose",
        "healthy": healthy,
        "nav": {"robot1": j1, "robot2": j2},
        "cancel_robot1_exit": cancel_ec,
        "wait": {"robot1": out1, "robot2": out2},
        "isolation_issues": isolation,
    }
    _slog("cancel_overlap_done", healthy=healthy, outcome_r1=o1, outcome_r2=o2)
    return result


def main() -> int:
    p = argparse.ArgumentParser(description="Stress parallel named missions (helper validation).")
    p.add_argument("--happy-iterations", type=int, default=5)
    p.add_argument("--per-goal-timeout", type=float, default=120.0)
    p.add_argument("--poll-interval", type=float, default=1.0)
    p.add_argument("--nav-call-timeout", type=float, default=60.0)
    p.add_argument("--wait-call-timeout", type=float, default=25.0)
    p.add_argument("--no-mixed", action="store_true", help="Skip mixed-validity scenario.")
    p.add_argument("--no-cancel-overlap", action="store_true", help="Skip pose+cancel scenario.")
    p.add_argument("--pose-x", type=float, default=6.0)
    p.add_argument("--pose-y", type=float, default=6.0)
    p.add_argument("--pose-yaw", type=float, default=0.0)
    args = p.parse_args()

    happy_steps = ["robot1:base", "robot2:test_goal"]
    expected_happy = [("robot1", "base"), ("robot2", "test_goal")]
    mixed_steps = ["robot1:base", "robot2:bad_location_xyz"]
    expected_mixed = [("robot1", "base"), ("robot2", "bad_location_xyz")]

    happy_results: List[Dict[str, Any]] = []
    isolation_issue_runs = 0
    timeouts_total = 0
    cancels_total = 0

    for i in range(args.happy_iterations):
        _slog("happy_iteration", index=i)
        ec, data = run_parallel(
            happy_steps,
            args.per_goal_timeout,
            args.poll_interval,
            args.nav_call_timeout,
            args.wait_call_timeout,
        )
        issues = isolation_check(data, expected_happy)
        t, c = count_timeouts_and_cancel(data)
        timeouts_total += t
        cancels_total += c
        if issues:
            isolation_issue_runs += 1
        happy_results.append(
            {
                "iteration": i,
                "exit_code": ec,
                "summary": data,
                "isolation_issues": issues,
            }
        )

    successful_runs = sum(
        1
        for h in happy_results
        if h["exit_code"] == 0
        and h["summary"].get("overall_outcome") == "success"
        and not h["isolation_issues"]
    )
    failed_runs = args.happy_iterations - successful_runs

    mixed_block: Optional[Dict[str, Any]] = None
    mixed_validity_failures = 0
    if not args.no_mixed:
        _slog("mixed_validity_start")
        ec, data = run_parallel(
            mixed_steps,
            min(60.0, args.per_goal_timeout),
            args.poll_interval,
            args.nav_call_timeout,
            args.wait_call_timeout,
        )
        issues = isolation_check(data, expected_mixed)
        t, c = count_timeouts_and_cancel(data)
        timeouts_total += t
        cancels_total += c
        # Expect overall failure but per-robot isolation still holds
        if issues:
            isolation_issue_runs += 1
        r1_ok = next(
            (s for s in data.get("steps") or [] if s.get("robot_id") == "robot1"), {}
        )
        invariant_r1 = r1_ok.get("step_outcome") == "succeeded"
        if not invariant_r1:
            mixed_validity_failures = 1
        mixed_block = {
            "exit_code": ec,
            "summary": data,
            "isolation_issues": issues,
            "invariant_robot1_succeeded": invariant_r1,
        }
        _slog("mixed_validity_done", exit_code=ec)

    cancel_block: Optional[Dict[str, Any]] = None
    if not args.no_cancel_overlap:
        cancel_block = scenario_cancel_overlap(
            args.pose_x,
            args.pose_y,
            args.pose_yaw,
            args.nav_call_timeout,
            args.per_goal_timeout,
            args.poll_interval,
            args.wait_call_timeout,
        )
        if cancel_block.get("wait", {}).get("robot1", {}).get("wait_json", {}).get("outcome") == "cancelled":
            cancels_total += 1

    total_runs = args.happy_iterations + (0 if args.no_mixed else 1) + (0 if args.no_cancel_overlap else 1)

    stress_ok = (
        successful_runs == args.happy_iterations
        and isolation_issue_runs == 0
        and (mixed_block is None or mixed_block.get("invariant_robot1_succeeded"))
        and (cancel_block is None or cancel_block.get("healthy"))
    )

    out = {
        "stress_summary": {
            "stress_pass": stress_ok,
            "total_runs": total_runs,
            "happy_iterations": args.happy_iterations,
            "successful_runs": successful_runs,
            "failed_runs": failed_runs,
            "mixed_validity_failures": mixed_validity_failures,
            "timeouts_observed": timeouts_total,
            "cancellations_observed": cancels_total,
            "isolation_issue_runs": isolation_issue_runs,
        },
        "happy_path_iterations": happy_results,
        "mixed_validity": mixed_block,
        "cancel_overlap": cancel_block,
    }
    print(json.dumps(out, separators=(",", ":")))
    return 0 if stress_ok else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BrokenPipeError:
        raise SystemExit(1)
