#!/usr/bin/env python3
"""
Direct smoke test for MissionClient against a running mission bridge.

Prerequisites (Ubuntu / ROS 2):
  - Workspace built and sourced (colcon build + source install/setup.bash)
  - mission_bridge_node running with Nav2 for robot1_ns (e.g. fully_integrated_swarm)

Run from workspace root or repo root:
  python3 scripts/validate_mission_client_ros.py
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.client import MissionClient  # noqa: E402


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


def _state_failed(st: dict) -> bool:
    if st.get("status") == "failed":
        return True
    if st.get("status") == "failure":
        return True
    return False


def main() -> int:
    client: MissionClient | None = None
    code = 0
    try:
        client = MissionClient()
        nav = client.navigate_to_named_location("robot1", "base")
        print(
            json.dumps({"step": "navigate_to_named_location", "result": nav}, separators=(",", ":"))
        )
        if _nav_failed(nav):
            code = 1
            return code

        gid = str(nav["goal_id"]).strip()
        st = client.get_navigation_state("robot1", gid)
        print(
            json.dumps({"step": "get_navigation_state", "result": st}, separators=(",", ":"))
        )
        if _state_failed(st):
            code = 1
        return code
    except Exception as exc:
        print(json.dumps({"step": "error", "error": str(exc)}, separators=(",", ":")))
        return 1
    finally:
        if client is not None:
            try:
                client.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
