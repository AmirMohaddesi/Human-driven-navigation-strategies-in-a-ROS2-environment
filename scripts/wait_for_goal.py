#!/usr/bin/env python3
"""
Poll mission-agent ROS query-state until a terminal navigation state or timeout.

Requires a ROS environment where `ros2 run multi_robot_mission_stack mission-agent` works.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from typing import Any, Dict, Optional, Tuple

SUCCESS_OUTCOMES = frozenset({"succeeded", "cancelled"})


def _stderr_log(**payload: Any) -> None:
    line = json.dumps(
        {"component": "wait_for_goal", **payload},
        separators=(",", ":"),
        default=str,
    )
    sys.stderr.write(line + "\n")


def terminal_outcome(payload: Dict[str, Any]) -> Optional[str]:
    """
    If this query-state payload is terminal, return a normalized outcome name;
    otherwise None (keep polling).
    """
    nav = str(payload.get("nav_status", "") or "").strip().lower()
    status = str(payload.get("status", "") or "").strip().lower()
    msg = str(payload.get("message", "") or "").lower()

    if nav in ("succeeded", "cancelled", "failed", "rejected", "not_found"):
        return nav

    if status in ("failed", "failure"):
        if "not found" in msg:
            return "not_found"
        return "failed"

    return None


def parse_agent_json(stdout: str) -> Dict[str, Any]:
    """Extract the last line that looks like mission-agent JSON."""
    for line in reversed(stdout.splitlines()):
        s = line.strip()
        if s.startswith("{") and s.endswith("}"):
            return json.loads(s)
    raise ValueError("no JSON object in stdout")


def run_query_state(robot_id: str, goal_id: str, call_timeout: float) -> Tuple[int, str, str]:
    cmd = [
        "ros2",
        "run",
        "multi_robot_mission_stack",
        "mission-agent",
        "--ros",
        "query-state",
        "--robot-id",
        robot_id,
        "--goal-id",
        goal_id,
    ]
    proc = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=call_timeout,
        check=False,
    )
    return proc.returncode, proc.stdout or "", proc.stderr or ""


def main() -> int:
    parser = argparse.ArgumentParser(description="Wait for a navigation goal via query-state polling.")
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--goal-id", required=True)
    parser.add_argument("--timeout", type=float, default=30.0, help="Seconds wall-clock.")
    parser.add_argument("--poll-interval", type=float, default=1.0, help="Seconds between polls.")
    parser.add_argument(
        "--call-timeout",
        type=float,
        default=25.0,
        help="Max seconds per single ros2/mission-agent invocation.",
    )
    args = parser.parse_args()

    robot_id = str(args.robot_id).strip()
    goal_id = str(args.goal_id).strip()
    _stderr_log(
        event="wait_started",
        robot_id=robot_id,
        goal_id=goal_id,
        timeout=float(args.timeout),
        poll_interval=float(args.poll_interval),
    )
    start = time.monotonic()
    deadline = start + float(args.timeout)
    polls = 0
    last_payload: Optional[Dict[str, Any]] = None

    try:
        while time.monotonic() < deadline:
            polls += 1
            try:
                code, out, err = run_query_state(robot_id, goal_id, args.call_timeout)
            except subprocess.TimeoutExpired:
                out_obj: Dict[str, Any] = {
                    "outcome": "failed",
                    "robot_id": robot_id,
                    "goal_id": goal_id,
                    "reason": "query_subprocess_timeout",
                    "polls": polls,
                    "elapsed_sec": round(time.monotonic() - start, 3),
                }
                _stderr_log(event="wait_finished", **out_obj)
                print(json.dumps(out_obj, separators=(",", ":")))
                return 1

            try:
                payload = parse_agent_json(out)
            except (json.JSONDecodeError, ValueError):
                reason = "query_command_failed" if code != 0 else "invalid_query_json"
                out_obj = {
                    "outcome": "failed",
                    "robot_id": robot_id,
                    "goal_id": goal_id,
                    "reason": reason,
                    "returncode": code,
                    "stdout": (out or "")[:500],
                    "stderr": (err or "")[:500],
                    "polls": polls,
                    "elapsed_sec": round(time.monotonic() - start, 3),
                }
                _stderr_log(event="wait_finished", **out_obj)
                print(json.dumps(out_obj, separators=(",", ":")))
                return 1

            last_payload = payload
            oc = terminal_outcome(payload)
            elapsed = round(time.monotonic() - start, 3)
            if oc is not None:
                out_obj = {
                    "outcome": oc,
                    "robot_id": robot_id,
                    "goal_id": goal_id,
                    "status": payload.get("status", ""),
                    "nav_status": payload.get("nav_status", ""),
                    "message": payload.get("message", ""),
                    "polls": polls,
                    "elapsed_sec": elapsed,
                }
                _stderr_log(event="wait_finished", **out_obj)
                print(json.dumps(out_obj, separators=(",", ":")))
                return 0 if oc in SUCCESS_OUTCOMES else 1

            time.sleep(max(0.1, float(args.poll_interval)))

        lp = last_payload or {}
        out_obj = {
            "outcome": "timeout",
            "robot_id": robot_id,
            "goal_id": goal_id,
            "last_status": lp.get("status", ""),
            "last_nav_status": lp.get("nav_status", ""),
            "last_message": lp.get("message", ""),
            "polls": polls,
            "elapsed_sec": round(time.monotonic() - start, 3),
        }
        _stderr_log(event="wait_finished", **out_obj)
        print(json.dumps(out_obj, separators=(",", ":")))
        return 1
    except BrokenPipeError:
        return 0
    except Exception as exc:
        err_obj = {
            "outcome": "failed",
            "robot_id": robot_id,
            "goal_id": goal_id,
            "reason": "internal_error",
            "message": str(exc),
        }
        _stderr_log(event="wait_finished", **err_obj)
        print(json.dumps(err_obj, separators=(",", ":")))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
