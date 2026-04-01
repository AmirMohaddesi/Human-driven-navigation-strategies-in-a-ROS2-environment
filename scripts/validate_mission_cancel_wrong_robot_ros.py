#!/usr/bin/env python3
"""
Live ROS validator: cancel_navigation wrong-robot ownership (MissionClient, V2.0.1).

Prerequisites:
  - source install/setup.bash
  - mission_bridge_node running (e.g. fully_integrated_swarm)

Run:
  python3 scripts/validate_mission_cancel_wrong_robot_ros.py

Flow: navigate_to_named_location on robot1 -> cancel_navigation on robot2 with robot1's goal_id.

Expected bridge contract:
  - status: failure
  - nav_status: wrong_robot
  - message: Goal id belongs to another robot
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.client import MissionClient  # noqa: E402

OWNER_ROBOT = "robot1"
OTHER_ROBOT = "robot2"
LOCATION = "base"

EXPECTED_STATUS = "failure"
EXPECTED_NAV_STATUS = "wrong_robot"
EXPECTED_MESSAGE = "Goal id belongs to another robot"


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


def _matches_wrong_robot_contract(res: dict) -> bool:
    st = str(res.get("status", "") or "").strip()
    nav = str(res.get("nav_status", "") or "").strip().lower()
    msg = str(res.get("message", "") or "").strip()
    return st == EXPECTED_STATUS and nav == EXPECTED_NAV_STATUS and msg == EXPECTED_MESSAGE


def main() -> int:
    client: MissionClient | None = None
    try:
        client = MissionClient()
        nav = client.navigate_to_named_location(OWNER_ROBOT, LOCATION)
        print(
            json.dumps(
                {"step": "navigate_to_named_location", "result": nav},
                separators=(",", ":"),
            ),
            flush=True,
        )
        if _nav_failed(nav):
            return 1

        gid = str(nav["goal_id"]).strip()
        cancel = client.cancel_navigation(OTHER_ROBOT, gid)
        print(
            json.dumps({"step": "cancel_navigation_wrong_robot", "result": cancel}, separators=(",", ":")),
            flush=True,
        )
        return 0 if _matches_wrong_robot_contract(cancel) else 1
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
