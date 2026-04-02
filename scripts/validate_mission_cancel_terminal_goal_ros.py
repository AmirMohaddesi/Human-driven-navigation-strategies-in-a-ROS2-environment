#!/usr/bin/env python3
"""
Live ROS validator: terminal-goal cancel contract (V2.0.2 slice, MissionClient path).

Prerequisites:
  - source install/setup.bash
  - mission_bridge_node + Nav2 for robot1_ns (e.g. fully_integrated_swarm)

Run:
  python3 scripts/validate_mission_cancel_terminal_goal_ros.py

Flow:
  1. navigate_to_named_location(robot1, base) — require non-failure + non-empty goal_id
  2. Poll get_navigation_state until nav_status is terminal (succeeded | cancelled | failed),
     or timeout (conservative bound below)
  3. cancel_navigation(robot1, goal_id) — require V2.0.2 terminal contract:
     status success, nav_status not_cancellable, exact message

Terminal-cancel contract (frozen for this milestone):
  - status: "success"
  - nav_status: "not_cancellable"
  - message: "Goal is already complete"
"""

from __future__ import annotations

import json
import os
import sys
import time

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.client import MissionClient  # noqa: E402

ROBOT_ID = "robot1"
LOCATION = "base"

# Conservative: integrated sim + Nav2 may be slow; fail clearly if never terminal
TERMINAL_POLL_INTERVAL_SEC = 1.0
TERMINAL_WAIT_MAX_SEC = 300.0

TERMINAL_NAV_STATUSES = frozenset({"succeeded", "cancelled", "failed"})

EXPECTED_CANCEL_STATUS = "success"
EXPECTED_CANCEL_NAV_STATUS = "not_cancellable"
EXPECTED_CANCEL_MESSAGE = "Goal is already complete"


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


def _is_terminal_query(st: dict) -> bool:
    if str(st.get("status", "") or "").strip().lower() != "success":
        return False
    nav = str(st.get("nav_status", "") or "").strip().lower()
    return nav in TERMINAL_NAV_STATUSES


def _terminal_cancel_ok(cancel: dict) -> bool:
    st = str(cancel.get("status", "") or "").strip()
    nav = str(cancel.get("nav_status", "") or "").strip()
    msg = str(cancel.get("message", "") or "").strip()
    return (
        st == EXPECTED_CANCEL_STATUS
        and nav == EXPECTED_CANCEL_NAV_STATUS
        and msg == EXPECTED_CANCEL_MESSAGE
    )


def main() -> int:
    client: MissionClient | None = None
    try:
        client = MissionClient()

        nav = client.navigate_to_named_location(ROBOT_ID, LOCATION)
        print(
            json.dumps({"step": "navigate_to_named_location", "result": nav}, separators=(",", ":")),
            flush=True,
        )
        if _nav_failed(nav):
            return 1

        gid = str(nav["goal_id"]).strip()

        deadline = time.monotonic() + TERMINAL_WAIT_MAX_SEC
        polls = 0
        while time.monotonic() < deadline:
            st = client.get_navigation_state(ROBOT_ID, gid)
            polls += 1
            print(
                json.dumps(
                    {
                        "step": "poll_until_terminal",
                        "poll_index": polls,
                        "result": st,
                        "max_wait_sec": TERMINAL_WAIT_MAX_SEC,
                    },
                    separators=(",", ":"),
                ),
                flush=True,
            )
            if _is_terminal_query(st):
                print(
                    json.dumps(
                        {
                            "step": "terminal_observed",
                            "poll_index": polls,
                            "nav_status": str(st.get("nav_status", "")),
                        },
                        separators=(",", ":"),
                    ),
                    flush=True,
                )
                break
            time.sleep(TERMINAL_POLL_INTERVAL_SEC)
        else:
            print(
                json.dumps(
                    {
                        "step": "terminal_wait_timeout",
                        "error": f"nav_status not in {sorted(TERMINAL_NAV_STATUSES)} within {TERMINAL_WAIT_MAX_SEC}s",
                        "polls": polls,
                    },
                    separators=(",", ":"),
                ),
                flush=True,
            )
            return 1

        cancel = client.cancel_navigation(ROBOT_ID, gid)
        print(json.dumps({"step": "cancel_navigation_after_terminal", "result": cancel}, separators=(",", ":")), flush=True)
        if not _terminal_cancel_ok(cancel):
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
