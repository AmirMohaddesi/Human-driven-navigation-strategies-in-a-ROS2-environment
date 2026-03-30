#!/usr/bin/env python3
"""
Smoke-test inspect_team_named_mission_specs_checked (pure Python, no ROS).

  python3 scripts/validate_coordinator_batch_inspect_checked.py
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.coordinator import inspect_team_named_mission_specs_checked  # noqa: E402


def main() -> int:
    specs = [
        {"mode": "sequence", "steps": [{"robot_id": "r1", "location_name": "a"}]},
        {
            "mode": "parallel",
            "steps": [
                {"robot_id": "r1", "location_name": "b"},
                {"robot_id": "r2", "location_name": "c"},
            ],
        },
    ]
    blob = inspect_team_named_mission_specs_checked(specs)
    print(json.dumps(blob, separators=(",", ":")))
    return 0 if blob.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
