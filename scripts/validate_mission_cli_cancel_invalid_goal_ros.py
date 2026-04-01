#!/usr/bin/env python3
"""
Live ROS validator: mission-agent CLI cancel with unknown goal_id (v1.1 edge).

Prerequisites:
  - source install/setup.bash
  - mission_bridge_node running (e.g. fully_integrated_swarm)

Run:
  python3 scripts/validate_mission_cli_cancel_invalid_goal_ros.py

Invokes: mission-agent --ros cancel --robot-id robot1 --goal-id bogus-goal-id-for-validator
(or python3 -m multi_robot_mission_stack.agent.cli …).

Expected JSON contract (same as client/facade invalid-goal validators):
  - status: failure
  - nav_status: not_found
  - message: Goal id not found for this robot

CLI note: _exit_code_from_result treats ``failure`` as non-success, so the process typically
exits 1 even when JSON is correct. This validator passes on JSON contract only; it prints
``cli_exit_code`` for operators.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from typing import List, Tuple

BOGUS_GOAL_ID = "bogus-goal-id-for-validator"
ROBOT_ID = "robot1"

EXPECTED_STATUS = "failure"
EXPECTED_NAV_STATUS = "not_found"
EXPECTED_MESSAGE = "Goal id not found for this robot"


def _cli_argv() -> List[str]:
    if shutil.which("mission-agent"):
        return ["mission-agent"]
    return [sys.executable, "-m", "multi_robot_mission_stack.agent.cli"]


def _run(args: List[str]) -> Tuple[int, str, str]:
    cmd = _cli_argv() + args
    r = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        env=os.environ.copy(),
        cwd=os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
    )
    return r.returncode, (r.stdout or "").strip(), (r.stderr or "").strip()


def _parse_json_line(stdout: str) -> dict:
    line = stdout.strip().splitlines()[-1] if stdout.strip() else ""
    try:
        return json.loads(line)
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON stdout: {line!r} ({exc})") from exc


def _matches_contract(res: dict) -> bool:
    st = str(res.get("status", "") or "").strip()
    nav = str(res.get("nav_status", "") or "").strip().lower()
    msg = str(res.get("message", "") or "").strip()
    return st == EXPECTED_STATUS and nav == EXPECTED_NAV_STATUS and msg == EXPECTED_MESSAGE


def main() -> int:
    ros = ["--ros"]
    print("[cli] cancel (invalid goal) …", flush=True)
    ec, out, err = _run(
        ros + ["cancel", "--robot-id", ROBOT_ID, "--goal-id", BOGUS_GOAL_ID]
    )
    print(out, flush=True)
    if err:
        print(err, file=sys.stderr, flush=True)
    try:
        parsed = _parse_json_line(out)
    except ValueError as exc:
        print(json.dumps({"step": "error", "phase": "parse", "error": str(exc)}, separators=(",", ":")))
        return 1
    print(
        json.dumps(
            {
                "step": "cancel_invalid_goal_parsed",
                "result": parsed,
                "cli_exit_code": ec,
            },
            separators=(",", ":"),
        ),
        flush=True,
    )
    if not _matches_contract(parsed):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
