#!/usr/bin/env python3
"""
Live ROS validator: facade cancel with unknown goal_id (v1.1 edge).

Prerequisites:
  - source install/setup.bash
  - mission_bridge_node running (e.g. fully_integrated_swarm)

Run:
  python3 scripts/validate_mission_agent_facade_cancel_invalid_goal_ros.py

Expected contract (must match validate_mission_cancel_invalid_goal_ros.py / bridge):
  - status: failure
  - nav_status: not_found
  - message: Goal id not found for this robot

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

BOGUS_GOAL_ID = "bogus-goal-id-for-validator"
ROBOT_ID = "robot1"

EXPECTED_STATUS = "failure"
EXPECTED_NAV_STATUS = "not_found"
EXPECTED_MESSAGE = "Goal id not found for this robot"


def _matches_contract(res: dict) -> bool:
    st = str(res.get("status", "") or "").strip()
    nav = str(res.get("nav_status", "") or "").strip().lower()
    msg = str(res.get("message", "") or "").strip()
    return st == EXPECTED_STATUS and nav == EXPECTED_NAV_STATUS and msg == EXPECTED_MESSAGE


def main() -> int:
    facade: MissionAgentFacade | None = None
    try:
        facade = MissionAgentFacade.with_ros()
        cmd = {
            "type": "cancel",
            "target": "navigation",
            "robot_id": ROBOT_ID,
            "goal_id": BOGUS_GOAL_ID,
        }
        out = facade.handle_command(cmd)
        print(json.dumps({"step": "handle_command_cancel_invalid_goal", "result": out}, separators=(",", ":")))
        return 0 if _matches_contract(out) else 1
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
