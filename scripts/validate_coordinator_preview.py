#!/usr/bin/env python3
"""
Smoke-test preview_team_named_mission_spec (no ROS).

  python3 scripts/validate_coordinator_preview.py
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.coordinator import preview_team_named_mission_spec  # noqa: E402


def main() -> int:
    spec = {
        "mode": "parallel",
        "steps": [
            {"robot_id": "robot1", "location_name": "base"},
            {"robot_id": "robot2", "location_name": "test_goal"},
        ],
        "options": {"max_workers": 2},
    }
    blob = preview_team_named_mission_spec(spec)
    print(json.dumps(blob, separators=(",", ":")))
    return 0 if blob.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
