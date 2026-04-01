#!/usr/bin/env python3
"""
Live ROS validator: ownership-safe cancel flow (MissionClient, V2.1 parity).

Prerequisites:
  - source install/setup.bash
  - mission_bridge_node running (e.g. fully_integrated_swarm)

Run:
  python3 scripts/validate_mission_client_ownership_cancel_ros.py

Flow: navigate_to_named_location robot1/base -> cancel_navigation robot2 (wrong_robot)
      -> cancel_navigation robot1 (success cancel). Same matrix as coordinator/CLI validators.
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

WRONG_EXPECTED_STATUS = "failure"
WRONG_EXPECTED_NAV_STATUS = "wrong_robot"
WRONG_EXPECTED_MESSAGE = "Goal id belongs to another robot"


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
    client: MissionClient | None = None
    try:
        client = MissionClient()
        nav = client.navigate_to_named_location(OWNER_ROBOT, LOCATION)
        print(
            json.dumps({"step": "navigate_to_named_location", "result": nav}, separators=(",", ":")),
            flush=True,
        )
        if _nav_failed(nav):
            return 1

        gid = str(nav["goal_id"]).strip()

        cancel_wrong = client.cancel_navigation(OTHER_ROBOT, gid)
        print(
            json.dumps(
                {"step": "cancel_navigation_wrong_robot", "result": cancel_wrong},
                separators=(",", ":"),
            ),
            flush=True,
        )
        if not _matches_wrong_robot_contract(cancel_wrong):
            return 1

        cancel_owner = client.cancel_navigation(OWNER_ROBOT, gid)
        print(
            json.dumps({"step": "cancel_navigation_owner", "result": cancel_owner}, separators=(",", ":")),
            flush=True,
        )
        if not _owner_cancel_ok(cancel_owner):
            return 1

        q = client.get_navigation_state(OWNER_ROBOT, gid)
        print(
            json.dumps({"step": "get_navigation_state_optional_observe", "result": q}, separators=(",", ":")),
            flush=True,
        )

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
