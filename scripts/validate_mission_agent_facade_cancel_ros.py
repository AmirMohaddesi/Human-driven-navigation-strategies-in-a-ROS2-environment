#!/usr/bin/env python3
"""
Live ROS validator: MissionAgentFacade cancel path (v1).

Prerequisites:
  - source install/setup.bash
  - mission_bridge_node + Nav2 for robot1_ns (e.g. fully_integrated_swarm)

Run:
  python3 scripts/validate_mission_agent_facade_cancel_ros.py

Flow: with_ros() -> navigate (named_location) -> cancel (navigation) -> query (navigation_state).

Same minimal contract as validate_mission_cancel_ros.py (MissionClient), but through adapter + policy + graph + tools.
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.agent import MissionAgentFacade  # noqa: E402


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
    facade: MissionAgentFacade | None = None
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
        if _nav_failed(nav):
            return 1

        gid = str(nav.get("goal_id", "")).strip()
        if not gid:
            print(json.dumps({"step": "error", "error": "no goal_id"}, separators=(",", ":")))
            return 1

        cancel_cmd = {
            "type": "cancel",
            "target": "navigation",
            "robot_id": "robot1",
            "goal_id": gid,
        }
        cancel = facade.handle_command(cancel_cmd)
        print(json.dumps({"step": "handle_command_cancel", "result": cancel}, separators=(",", ":")))
        if _cancel_failed(cancel):
            return 1

        q_cmd = {
            "type": "query",
            "target": "navigation_state",
            "robot_id": "robot1",
            "goal_id": gid,
        }
        st = facade.handle_command(q_cmd)
        print(
            json.dumps({"step": "handle_command_query_after_cancel", "result": st}, separators=(",", ":"))
        )
        if _query_after_cancel_failed(st):
            return 1
        return 0
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
