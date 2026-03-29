#!/usr/bin/env python3
"""
Smoke-test assign_team_named_mission against a live stack.

Prerequisites: workspace sourced; mission_bridge_node + multi-robot stack running.

  python3 scripts/validate_coordinator_dispatch_ros.py
  python3 scripts/validate_coordinator_dispatch_ros.py --mode parallel
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, List

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.coordinator import assign_team_named_mission  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description="Validate coordinator assign_team_named_mission (ROS).")
    p.add_argument("--mode", choices=("sequence", "parallel"), default="sequence")
    p.add_argument("--bridge-node", default="mission_bridge_node")
    p.add_argument("--per-goal-timeout", type=float, default=120.0)
    p.add_argument("--poll-interval", type=float, default=1.0)
    p.add_argument("--max-workers", type=int, default=None)
    args = p.parse_args()

    mission: List[Dict[str, str]] = [
        {"robot_id": "robot1", "location_name": "base"},
        {"robot_id": "robot2", "location_name": "test_goal"},
    ]

    blob = assign_team_named_mission(
        mission,
        str(args.mode),
        per_goal_timeout_sec=float(args.per_goal_timeout),
        poll_interval_sec=float(args.poll_interval),
        max_workers=args.max_workers,
        bridge_node_name=str(args.bridge_node),
    )
    print(json.dumps(blob, separators=(",", ":")))
    return 0 if blob.get("overall_outcome") == "success" else 1


if __name__ == "__main__":
    raise SystemExit(main())
