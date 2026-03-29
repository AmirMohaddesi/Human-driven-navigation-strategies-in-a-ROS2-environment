#!/usr/bin/env python3
"""
Smoke-test Layer B coordinator assign_named_sequence against a live stack.

Prerequisites: workspace sourced; mission_bridge_node + multi-robot stack running.

  python3 scripts/validate_coordinator_sequence_ros.py
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.coordinator import assign_named_sequence  # noqa: E402


def _emit(obj: Dict[str, Any]) -> None:
    print(json.dumps(obj, separators=(",", ":")))


def main() -> int:
    p = argparse.ArgumentParser(description="Validate coordinator assign_named_sequence (ROS).")
    p.add_argument("--bridge-node", default="mission_bridge_node")
    p.add_argument("--per-goal-timeout", type=float, default=120.0)
    p.add_argument("--poll-interval", type=float, default=1.0)
    p.add_argument(
        "--continue-on-failure",
        action="store_true",
        help="Forward to assign_named_sequence.",
    )
    args = p.parse_args()

    mission: List[Dict[str, str]] = [
        {"robot_id": "robot1", "location_name": "base"},
        {"robot_id": "robot2", "location_name": "test_goal"},
    ]

    summary = assign_named_sequence(
        mission,
        per_goal_timeout_sec=float(args.per_goal_timeout),
        poll_interval_sec=float(args.poll_interval),
        continue_on_failure=bool(args.continue_on_failure),
        bridge_node_name=str(args.bridge_node),
    )
    ok = summary.get("overall_outcome") == "success"
    _emit({"ok": ok, "summary": summary})
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
