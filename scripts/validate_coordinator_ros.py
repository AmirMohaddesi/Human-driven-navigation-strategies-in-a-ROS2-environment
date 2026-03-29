#!/usr/bin/env python3
"""
Smoke-test Layer B coordinator (assign_named_navigation) against a live stack.

Prerequisites: workspace sourced; mission_bridge_node + robot/Nav2 running.

  python3 scripts/validate_coordinator_ros.py
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

from multi_robot_mission_stack.coordinator import assign_named_navigation  # noqa: E402


def _emit(obj: Dict[str, Any]) -> None:
    print(json.dumps(obj, separators=(",", ":")))


def main() -> int:
    p = argparse.ArgumentParser(description="Validate coordinator assign_named_navigation (ROS).")
    p.add_argument("--robot-id", default="robot1")
    p.add_argument("--location-name", default="base")
    p.add_argument("--bridge-node", default="mission_bridge_node")
    p.add_argument("--per-goal-timeout", type=float, default=120.0)
    p.add_argument("--poll-interval", type=float, default=1.0)
    args = p.parse_args()

    result = assign_named_navigation(
        args.robot_id,
        args.location_name,
        per_goal_timeout_sec=float(args.per_goal_timeout),
        poll_interval_sec=float(args.poll_interval),
        bridge_node_name=str(args.bridge_node),
    )
    ok = str(result.get("outcome", "")).lower() == "succeeded"
    _emit({"ok": ok, "result": result})
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
