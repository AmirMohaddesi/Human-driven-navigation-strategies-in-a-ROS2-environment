#!/usr/bin/env python3
"""
Smoke-test run_team_named_mission_spec against a live stack.

Prerequisites: workspace sourced; mission_bridge_node + multi-robot stack running.

  python3 scripts/validate_coordinator_spec_runner.py
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.coordinator import run_team_named_mission_spec  # noqa: E402


def main() -> int:
    spec = {
        "mode": "sequence",
        "steps": [
            {"robot_id": "robot1", "location_name": "base"},
            {"robot_id": "robot2", "location_name": "test_goal"},
        ],
        "options": {
            "per_goal_timeout_sec": 120.0,
            "poll_interval_sec": 1.0,
            "continue_on_failure": False,
            "bridge_node_name": "mission_bridge_node",
        },
    }
    blob = run_team_named_mission_spec(spec)
    print(json.dumps(blob, separators=(",", ":")))
    return 0 if blob.get("overall_outcome") == "success" else 1


if __name__ == "__main__":
    raise SystemExit(main())
