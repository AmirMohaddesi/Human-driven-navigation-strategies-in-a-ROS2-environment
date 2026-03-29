#!/usr/bin/env python3
"""
Smoke-test preflight_team_named_mission_spec (no ROS).

  python3 scripts/validate_coordinator_preflight.py
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.coordinator import preflight_team_named_mission_spec  # noqa: E402


def main() -> int:
    spec = {
        "mode": "parallel",
        "steps": [
            {"robot_id": "robot1", "location_name": "base"},
            {"robot_id": "robot2", "location_name": "test_goal"},
        ],
        "options": {"poll_interval_sec": 1.0, "max_workers": 2},
    }
    blob = preflight_team_named_mission_spec(spec)
    print(json.dumps(blob, separators=(",", ":")))
    return 0 if blob.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
