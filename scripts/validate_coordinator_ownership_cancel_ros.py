#!/usr/bin/env python3
"""
Live ROS validator: ownership-safe cancel flow (V2.1 coordinator-prep slice).

Navigate via ``MissionAgentFacade`` (same command shape as ``assign_named_navigation``).
Cancel steps via Layer B ``cancel_navigation_with_ros`` (delegates to
``cancel_navigation_via_facade`` per session). Bridge remains ownership authority.

Prerequisites:
  - source install/setup.bash
  - mission_bridge_node running (e.g. fully_integrated_swarm)

Run:
  python3 scripts/validate_coordinator_ownership_cancel_ros.py

Flow:
  1. Navigate robot1 -> base (non-failure status, non-empty goal_id)
  2. Cancel as robot2 with robot1's goal_id -> wrong_robot contract
  3. Cancel as robot1 with same goal_id -> success cancel (nav_status cancelling | not_cancellable)
  4. Optional: query-state robot1 + goal_id (log only; no new query contract checks)
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.agent import MissionAgentFacade  # noqa: E402
from multi_robot_mission_stack.agent.sequence_utils import _navigate_named_command  # noqa: E402
from multi_robot_mission_stack.coordinator import cancel_navigation_with_ros  # noqa: E402

OWNER_ROBOT = "robot1"
OTHER_ROBOT = "robot2"
LOCATION = "base"

WRONG_EXPECTED_STATUS = "failure"
WRONG_EXPECTED_NAV_STATUS = "wrong_robot"
WRONG_EXPECTED_MESSAGE = "Goal id belongs to another robot"


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
    return (
        st == WRONG_EXPECTED_STATUS
        and nav == WRONG_EXPECTED_NAV_STATUS
        and msg == WRONG_EXPECTED_MESSAGE
    )


def _correct_owner_cancel_ok(res: dict) -> bool:
    st = str(res.get("status", "") or "").strip().lower()
    if st in ("failed", "failure"):
        return False
    nav = str(res.get("nav_status", "") or "").strip().lower()
    return nav in ("cancelling", "not_cancellable")


def main() -> int:
    facade: MissionAgentFacade | None = None
    try:
        facade = MissionAgentFacade.with_ros()

        nav = facade.handle_command(_navigate_named_command(OWNER_ROBOT, LOCATION))
        print(
            json.dumps(
                {"step": "navigate_named_location_facade_session", "result": nav},
                separators=(",", ":"),
            ),
            flush=True,
        )
        if _navigate_failed(nav):
            return 1

        gid = str(nav["goal_id"]).strip()

        out_wrong = cancel_navigation_with_ros(OTHER_ROBOT, gid)
        print(
            json.dumps(
                {"step": "cancel_navigation_with_ros_wrong_robot", "result": out_wrong},
                separators=(",", ":"),
            ),
            flush=True,
        )
        if not _matches_wrong_robot_contract(out_wrong):
            return 1

        out_ok = cancel_navigation_with_ros(OWNER_ROBOT, gid)
        print(
            json.dumps(
                {"step": "cancel_navigation_with_ros_owner", "result": out_ok},
                separators=(",", ":"),
            ),
            flush=True,
        )
        if not _correct_owner_cancel_ok(out_ok):
            return 1

        q_cmd = {
            "type": "query",
            "target": "navigation_state",
            "robot_id": OWNER_ROBOT,
            "goal_id": gid,
        }
        q_out = facade.handle_command(q_cmd)
        print(
            json.dumps({"step": "handle_command_query_optional_observe", "result": q_out}, separators=(",", ":")),
            flush=True,
        )

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
