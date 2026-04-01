#!/usr/bin/env python3
"""
Live ROS validator: mission-agent CLI ownership-safe cancel (V2.1 parity).

Prerequisites:
  - source install/setup.bash
  - mission_bridge_node running (e.g. fully_integrated_swarm)

Run:
  python3 scripts/validate_mission_cli_ownership_cancel_ros.py

Flow: navigate robot1 base -> cancel robot2 (wrong_robot) -> cancel robot1 (success cancel).
Same story as validate_coordinator_ownership_cancel_ros.py via CLI surface.

CLI policy: failure-shaped JSON may still exit nonzero; success cancel typically exits 0.
Validator is JSON-authoritative; prints cli_exit_code for each cancel step.
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

WRONG_EXPECTED_STATUS = "failure"
WRONG_EXPECTED_NAV_STATUS = "wrong_robot"
WRONG_EXPECTED_MESSAGE = "Goal id belongs to another robot"


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
    return (
        st == WRONG_EXPECTED_STATUS
        and nav == WRONG_EXPECTED_NAV_STATUS
        and msg == WRONG_EXPECTED_MESSAGE
    )


def _owner_cancel_ok(res: dict) -> bool:
    st = str(res.get("status", "") or "").strip().lower()
    if st in ("failed", "failure"):
        return False
    nav = str(res.get("nav_status", "") or "").strip().lower()
    return nav in ("cancelling", "not_cancellable")


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
    ec_w, out_w, err_w = _run(
        ros + ["cancel", "--robot-id", OTHER_ROBOT, "--goal-id", gid]
    )
    print(out_w, flush=True)
    if err_w:
        print(err_w, file=sys.stderr, flush=True)
    try:
        parsed_w = _parse_json_line(out_w, "cancel_wrong_robot")
    except ValueError as exc:
        print(json.dumps({"step": "error", "phase": "cancel_wrong_parse", "error": str(exc)}, separators=(",", ":")))
        return 1
    print(
        json.dumps(
            {
                "step": "cancel_wrong_robot_parsed",
                "result": parsed_w,
                "cli_exit_code": ec_w,
            },
            separators=(",", ":"),
        ),
        flush=True,
    )
    if not _matches_wrong_robot_contract(parsed_w):
        return 1

    print("[cli] cancel (owner) …", flush=True)
    ec_o, out_o, err_o = _run(
        ros + ["cancel", "--robot-id", OWNER_ROBOT, "--goal-id", gid]
    )
    print(out_o, flush=True)
    if err_o:
        print(err_o, file=sys.stderr, flush=True)
    try:
        parsed_o = _parse_json_line(out_o, "cancel_owner")
    except ValueError as exc:
        print(json.dumps({"step": "error", "phase": "cancel_owner_parse", "error": str(exc)}, separators=(",", ":")))
        return 1
    print(
        json.dumps(
            {
                "step": "cancel_owner_parsed",
                "result": parsed_o,
                "cli_exit_code": ec_o,
            },
            separators=(",", ":"),
        ),
        flush=True,
    )
    if not _owner_cancel_ok(parsed_o):
        return 1

    print("[cli] query-state (optional observe) …", flush=True)
    ec_q, out_q, err_q = _run(
        ros + ["query-state", "--robot-id", OWNER_ROBOT, "--goal-id", gid]
    )
    print(out_q, flush=True)
    if err_q:
        print(err_q, file=sys.stderr, flush=True)
    try:
        q_parsed = _parse_json_line(out_q, "query_observe")
    except ValueError as exc:
        print(
            json.dumps(
                {"step": "query_observe_parse_skipped", "error": str(exc)},
                separators=(",", ":"),
            ),
            flush=True,
        )
    else:
        print(
            json.dumps(
                {
                    "step": "query_observe_log_only",
                    "result": q_parsed,
                    "cli_exit_code": ec_q,
                },
                separators=(",", ":"),
            ),
            flush=True,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
