#!/usr/bin/env python3
"""
Cancel-path validator for the direct MissionClient layer (v1).

Prerequisites (Ubuntu / ROS 2):
  - Workspace built and sourced (colcon build + source install/setup.bash)
  - mission_bridge_node running with Nav2 for robot1_ns (e.g. fully_integrated_swarm)

Run from repository root:
  python3 scripts/validate_mission_cancel_ros.py

Flow: navigate_to_named_location -> cancel_navigation -> get_navigation_state (same goal_id).

Minimal v1 contract:
  - navigate: non-empty goal_id; status not failed/failure; nav_status not rejected (same as golden path)
  - cancel: status not failed/failure; nav_status in {cancelling, not_cancellable}
      (cancelling is primary; not_cancellable is accepted as timing-sensitive)
  - follow-up query: status == success; nav_status non-empty and not unknown / not_found

Stale-runtime safeguard: rebuild + fresh relaunch before trusting results after bridge changes.
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
    client: MissionClient | None = None
    try:
        client = MissionClient()
        nav = client.navigate_to_named_location("robot1", "base")
        print(
            json.dumps({"step": "navigate_to_named_location", "result": nav}, separators=(",", ":"))
        )
        if _nav_failed(nav):
            return 1

        gid = str(nav["goal_id"]).strip()
        cancel = client.cancel_navigation("robot1", gid)
        print(json.dumps({"step": "cancel_navigation", "result": cancel}, separators=(",", ":")))
        if _cancel_failed(cancel):
            return 1

        st = client.get_navigation_state("robot1", gid)
        print(
            json.dumps({"step": "get_navigation_state_after_cancel", "result": st}, separators=(",", ":"))
        )
        if _query_after_cancel_failed(st):
            return 1
        return 0
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
