#!/usr/bin/env python3
"""
Smoke-test summarize_team_named_mission_result (no ROS).

  python3 scripts/validate_coordinator_single_result_summary.py
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.coordinator import summarize_team_named_mission_result  # noqa: E402


def main() -> int:
    result = {
        "ok": True,
        "mode": "sequence",
        "overall_outcome": "success",
        "summary": {
            "overall_outcome": "success",
            "total_steps": 1,
            "steps_run": 1,
            "succeeded_count": 1,
            "failed_count": 0,
            "stopped_early": False,
            "continue_on_failure": False,
            "steps": [{"index": 0, "step_outcome": "succeeded"}],
        },
    }
    blob = summarize_team_named_mission_result(result)
    print(json.dumps(blob, separators=(",", ":")))
    return 0 if blob.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
