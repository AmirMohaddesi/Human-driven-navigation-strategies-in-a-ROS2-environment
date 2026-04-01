#!/usr/bin/env python3
"""
Live ROS validator: mission-agent CLI wrong-robot cancel (V2.0.1).

Prerequisites:
  - source install/setup.bash
  - mission_bridge_node running (e.g. fully_integrated_swarm)

Run:
  python3 scripts/validate_mission_cli_cancel_wrong_robot_ros.py

Invokes: navigate robot1 base -> cancel robot2 with robot1's goal_id
(mission-agent --ros … or python3 -m multi_robot_mission_stack.agent.cli …).

Expected cancel JSON contract (same as client/facade wrong-robot validators):
  - status: failure
  - nav_status: wrong_robot
  - message: Goal id belongs to another robot

CLI note: cancel returns status failure -> process typically exits nonzero even when JSON
is contract-correct. Validator passes on JSON contract for cancel; prints cli_exit_code.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from typing import List, Tuple

OWNER_ROBOT = "robot1"
OTHER_ROBOT = "robot2"
LOCATION = "base"

EXPECTED_STATUS = "failure"
EXPECTED_NAV_STATUS = "wrong_robot"
EXPECTED_MESSAGE = "Goal id belongs to another robot"


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


def _parse_json_line(stdout: str, step: str) -> dict:
    line = stdout.strip().splitlines()[-1] if stdout.strip() else ""
    try:
        return json.loads(line)
    except json.JSONDecodeError as exc:
        raise ValueError(f"{step}: invalid JSON stdout: {line!r} ({exc})") from exc


def _navigate_failed(nav: dict) -> bool:
    if nav.get("status") == "failed":
        return True
    if nav.get("status") == "failure":
        return True
    if nav.get("nav_status") == "rejected":
        return True
    if not (nav.get("goal_id") or "").strip():
        return True
    return False


def _matches_wrong_robot_contract(res: dict) -> bool:
    st = str(res.get("status", "") or "").strip()
    nav = str(res.get("nav_status", "") or "").strip().lower()
    msg = str(res.get("message", "") or "").strip()
    return st == EXPECTED_STATUS and nav == EXPECTED_NAV_STATUS and msg == EXPECTED_MESSAGE


def main() -> int:
    ros = ["--ros"]

    print("[cli] navigate …", flush=True)
    ec_n, out_n, err_n = _run(
        ros + ["navigate", "--robot-id", OWNER_ROBOT, "--location-name", LOCATION]
    )
    print(out_n, flush=True)
    if err_n:
        print(err_n, file=sys.stderr, flush=True)
    if ec_n != 0:
        print(
            json.dumps(
                {"step": "error", "phase": "navigate_exit", "exit_code": ec_n},
                separators=(",", ":"),
            )
        )
        return 1
    try:
        nav = _parse_json_line(out_n, "navigate")
    except ValueError as exc:
        print(json.dumps({"step": "error", "phase": "navigate_parse", "error": str(exc)}, separators=(",", ":")))
        return 1
    print(json.dumps({"step": "navigate_parsed", "result": nav}, separators=(",", ":")), flush=True)
    if _navigate_failed(nav):
        return 1

    gid = str(nav["goal_id"]).strip()

    print("[cli] cancel (wrong robot) …", flush=True)
    ec_c, out_c, err_c = _run(
        ros + ["cancel", "--robot-id", OTHER_ROBOT, "--goal-id", gid]
    )
    print(out_c, flush=True)
    if err_c:
        print(err_c, file=sys.stderr, flush=True)
    try:
        parsed = _parse_json_line(out_c, "cancel")
    except ValueError as exc:
        print(json.dumps({"step": "error", "phase": "cancel_parse", "error": str(exc)}, separators=(",", ":")))
        return 1
    print(
        json.dumps(
            {
                "step": "cancel_wrong_robot_parsed",
                "result": parsed,
                "cli_exit_code": ec_c,
            },
            separators=(",", ":"),
        ),
        flush=True,
    )
    if not _matches_wrong_robot_contract(parsed):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
