#!/usr/bin/env python3
"""
Live ROS validator: facade wrong-robot cancel (MissionAgentFacade.with_ros(), V2.0.1).

Prerequisites:
  - source install/setup.bash
  - mission_bridge_node running (e.g. fully_integrated_swarm)

Run:
  python3 scripts/validate_mission_agent_facade_cancel_wrong_robot_ros.py

Flow: navigate named_location robot1 base -> cancel navigation robot2 with robot1's goal_id.

Expected contract (must match bridge / validate_mission_cancel_wrong_robot_ros.py):
  - status: failure
  - nav_status: wrong_robot
  - message: Goal id belongs to another robot

Path: CommandAdapter -> MissionGraph -> MissionTools.cancel_navigation -> MissionClient.
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.agent import MissionAgentFacade  # noqa: E402

OWNER_ROBOT = "robot1"
OTHER_ROBOT = "robot2"
LOCATION = "base"

EXPECTED_STATUS = "failure"
EXPECTED_NAV_STATUS = "wrong_robot"
EXPECTED_MESSAGE = "Goal id belongs to another robot"


def _navigate_failed(nav: dict) -> bool:
    st = str(nav.get("status", "") or "").strip().lower()
    if st in ("failed", "failure"):
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
    facade: MissionAgentFacade | None = None
    try:
        facade = MissionAgentFacade.with_ros()
        nav_cmd = {
            "type": "navigate",
            "target": "named_location",
            "robot_id": OWNER_ROBOT,
            "location_name": LOCATION,
        }
        nav = facade.handle_command(nav_cmd)
        print(
            json.dumps({"step": "handle_command_navigate", "result": nav}, separators=(",", ":")),
            flush=True,
        )
        if _navigate_failed(nav):
            return 1

        gid = str(nav["goal_id"]).strip()
        cancel_cmd = {
            "type": "cancel",
            "target": "navigation",
            "robot_id": OTHER_ROBOT,
            "goal_id": gid,
        }
        out = facade.handle_command(cancel_cmd)
        print(
            json.dumps({"step": "handle_command_cancel_wrong_robot", "result": out}, separators=(",", ":")),
            flush=True,
        )
        return 0 if _matches_wrong_robot_contract(out) else 1
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
