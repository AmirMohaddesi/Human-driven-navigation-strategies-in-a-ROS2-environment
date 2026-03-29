#!/usr/bin/env python3
"""
Smoke-test normalize_team_named_mission_spec (no ROS).

  python3 scripts/validate_coordinator_normalize_spec.py
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.coordinator import normalize_team_named_mission_spec  # noqa: E402


def main() -> int:
    messy = {
        "mode": "  Parallel  ",
        "steps": [
            {"robot_id": "  robot1  ", "location_name": "  base  ", "note": "x"},
            {"robot_id": "robot2", "location_name": "test_goal"},
        ],
    }
    blob = normalize_team_named_mission_spec(messy["steps"], messy["mode"])
    print(json.dumps(blob, separators=(",", ":")))
    return 0 if blob.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
