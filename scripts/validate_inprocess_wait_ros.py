#!/usr/bin/env python3
"""
Validate in-process wait_for_terminal_navigation_state (MissionAgentFacade + query loop).

Prerequisites: workspace sourced; mission_bridge_node + stack running.

  python3 scripts/validate_inprocess_wait_ros.py
  python3 scripts/validate_inprocess_wait_ros.py --also-cancel
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.agent import MissionAgentFacade  # noqa: E402
from multi_robot_mission_stack.agent.wait_utils import (  # noqa: E402
    wait_for_terminal_navigation_state,
)

_SUCCESS = frozenset({"succeeded", "cancelled"})


def _emit(obj: Dict[str, Any]) -> None:
    print(json.dumps(obj, separators=(",", ":")))


def main() -> int:
    p = argparse.ArgumentParser(description="Validate in-process navigation wait (facade + wait_utils).")
    p.add_argument("--robot-id", default="robot1")
    p.add_argument("--location-name", default="base")
    p.add_argument("--bridge-node", default="mission_bridge_node")
    p.add_argument("--timeout", type=float, default=120.0)
    p.add_argument("--poll-interval", type=float, default=1.0)
    p.add_argument(
        "--also-cancel",
        action="store_true",
        help="Submit goal, cancel via facade, then wait for cancelled.",
    )
    args = p.parse_args()

    summary: Dict[str, Any] = {"ok": False, "phases": []}
    facade: MissionAgentFacade | None = None

    try:
        facade = MissionAgentFacade.with_ros(bridge_node_name=args.bridge_node)
    except Exception as exc:
        summary["error"] = {"phase": "facade_construct", "message": str(exc)}
        _emit(summary)
        return 1

    try:
        nav_cmd = {
            "type": "navigate",
            "target": "named_location",
            "robot_id": args.robot_id,
            "location_name": args.location_name,
        }
        nav = facade.handle_command(nav_cmd)
        summary["phases"].append({"phase": "navigate", "result": nav})

        st = str(nav.get("status", "")).lower()
        if st in ("failed", "failure") or not nav.get("goal_id"):
            summary["error"] = {"phase": "navigate", "message": "navigate did not yield goal_id"}
            _emit(summary)
            return 1

        gid = str(nav["goal_id"])

        if args.also_cancel:
            cancel = facade.handle_command(
                {
                    "type": "cancel",
                    "target": "navigation",
                    "robot_id": args.robot_id,
                    "goal_id": gid,
                }
            )
            summary["phases"].append({"phase": "cancel", "result": cancel})

        wait_res = wait_for_terminal_navigation_state(
            facade,
            args.robot_id,
            gid,
            timeout_sec=float(args.timeout),
            poll_interval_sec=float(args.poll_interval),
        )
        summary["phases"].append({"phase": "wait", "result": wait_res})
        summary["ok"] = wait_res.get("outcome") in _SUCCESS
        _emit(summary)
        return 0 if summary["ok"] else 1
    except Exception as exc:
        summary["error"] = {"phase": "unexpected", "message": str(exc)}
        _emit(summary)
        return 1
    finally:
        if facade is not None:
            try:
                facade.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
