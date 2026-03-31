#!/usr/bin/env python3
"""
Golden-path baseline validator for the facade layer (MissionAgentFacade.with_ros()).

Prerequisites (Ubuntu / ROS 2):
  - Workspace built and sourced
  - mission_bridge_node + robot/Nav2 stack running

Run:
  python3 scripts/validate_mission_agent_facade_ros.py

Baseline expectation:
  - in a healthy runtime, this should agree with the direct client validator
  - navigate returns a non-empty goal_id
  - query returns a non-failure / success-like status
  - process exits with code 0

Stale-runtime safeguard:
  - after Python bridge changes, rebuild and relaunch before trusting results
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.agent import MissionAgentFacade  # noqa: E402


def _cmd_failed(out: dict) -> bool:
    status = str(out.get("status", "") or "").strip().lower()
    if status in {"failed", "failure"}:
        return True
    nav_status = str(out.get("nav_status", "") or "").strip().lower()
    if nav_status in {"", "unknown", "not_found"}:
        return True
    return False


def main() -> int:
    facade: MissionAgentFacade | None = None
    code = 0
    try:
        facade = MissionAgentFacade.with_ros()
        nav_cmd = {
            "type": "navigate",
            "target": "named_location",
            "robot_id": "robot1",
            "location_name": "base",
        }
        nav = facade.handle_command(nav_cmd)
        print(json.dumps({"step": "handle_command_navigate", "result": nav}, separators=(",", ":")))
        if _cmd_failed(nav):
            return 1
        gid = nav.get("goal_id")
        if not gid:
            print(json.dumps({"step": "error", "error": "no goal_id"}, separators=(",", ":")))
            return 1

        q_cmd = {
            "type": "query",
            "target": "navigation_state",
            "robot_id": "robot1",
            "goal_id": str(gid),
        }
        st = facade.handle_command(q_cmd)
        print(json.dumps({"step": "handle_command_query", "result": st}, separators=(",", ":")))
        if _cmd_failed(st):
            code = 1
        return code
    except Exception as exc:
        print(json.dumps({"step": "error", "error": str(exc)}, separators=(",", ":")))
        return 1
    finally:
        if facade is not None:
            try:
                facade.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
