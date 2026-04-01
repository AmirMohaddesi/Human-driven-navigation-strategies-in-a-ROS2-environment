#!/usr/bin/env python3
"""
Live ROS validator: mission-agent CLI cancel path (v1).

Prerequisites:
  - source install/setup.bash (workspace overlay so ``mission-agent`` or package is available)
  - mission_bridge_node + Nav2 for robot1_ns

Run from repository root:
  python3 scripts/validate_mission_cli_cancel_ros.py

Invokes the real CLI: navigate -> cancel -> query-state (same contract as facade/client cancel validators).
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from typing import List, Tuple


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


def _nav_failed(nav: dict) -> bool:
    if nav.get("status") == "failed":
        return True
    if nav.get("status") == "failure":
        return True
    if nav.get("nav_status") == "rejected":
        return True
    if not (nav.get("goal_id") or "").strip():
        return True
    return False


def _cancel_failed(cancel: dict) -> bool:
    st = str(cancel.get("status", "") or "").strip().lower()
    if st in ("failed", "failure"):
        return True
    nav = str(cancel.get("nav_status", "") or "").strip().lower()
    if nav not in ("cancelling", "not_cancellable"):
        return True
    return False


def _query_after_cancel_failed(st: dict) -> bool:
    if str(st.get("status", "") or "").strip().lower() != "success":
        return True
    nav_status = str(st.get("nav_status", "") or "").strip().lower()
    if nav_status in {"", "unknown", "not_found"}:
        return True
    return False


def main() -> int:
    ros = ["--ros"]

    print("[cli] navigate …", flush=True)
    ec, out, err = _run(ros + ["navigate", "--robot-id", "robot1", "--location-name", "base"])
    print(out, flush=True)
    if err:
        print(err, file=sys.stderr, flush=True)
    if ec != 0:
        print(json.dumps({"step": "error", "phase": "navigate_exit", "exit_code": ec}, separators=(",", ":")))
        return 1
    try:
        nav = _parse_json_line(out, "navigate")
    except ValueError as exc:
        print(json.dumps({"step": "error", "phase": "navigate_parse", "error": str(exc)}, separators=(",", ":")))
        return 1
    print(json.dumps({"step": "navigate_parsed", "result": nav}, separators=(",", ":")))
    if _nav_failed(nav):
        return 1

    gid = str(nav["goal_id"]).strip()

    print("[cli] cancel …", flush=True)
    ec_c, out_c, err_c = _run(ros + ["cancel", "--robot-id", "robot1", "--goal-id", gid])
    print(out_c, flush=True)
    if err_c:
        print(err_c, file=sys.stderr, flush=True)
    if ec_c != 0:
        print(json.dumps({"step": "error", "phase": "cancel_exit", "exit_code": ec_c}, separators=(",", ":")))
        return 1
    try:
        cancel = _parse_json_line(out_c, "cancel")
    except ValueError as exc:
        print(json.dumps({"step": "error", "phase": "cancel_parse", "error": str(exc)}, separators=(",", ":")))
        return 1
    print(json.dumps({"step": "cancel_parsed", "result": cancel}, separators=(",", ":")))
    if _cancel_failed(cancel):
        return 1

    print("[cli] query-state …", flush=True)
    ec_q, out_q, err_q = _run(
        ros + ["query-state", "--robot-id", "robot1", "--goal-id", gid]
    )
    print(out_q, flush=True)
    if err_q:
        print(err_q, file=sys.stderr, flush=True)
    if ec_q != 0:
        print(json.dumps({"step": "error", "phase": "query_exit", "exit_code": ec_q}, separators=(",", ":")))
        return 1
    try:
        st = _parse_json_line(out_q, "query")
    except ValueError as exc:
        print(json.dumps({"step": "error", "phase": "query_parse", "error": str(exc)}, separators=(",", ":")))
        return 1
    print(json.dumps({"step": "query_after_cancel_parsed", "result": st}, separators=(",", ":")))
    if _query_after_cancel_failed(st):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
