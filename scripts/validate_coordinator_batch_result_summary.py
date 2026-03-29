#!/usr/bin/env python3
"""
Smoke-test summarize_team_named_mission_specs_result (no ROS).

  python3 scripts/validate_coordinator_batch_result_summary.py
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.coordinator import summarize_team_named_mission_specs_result  # noqa: E402


def main() -> int:
    batch_result = {
        "ok": True,
        "overall_outcome": "success",
        "total_specs": 2,
        "specs_run": 2,
        "succeeded_count": 2,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_batch_failure": False,
        "results": [
            {"index": 0, "ok": True, "mode": "sequence", "overall_outcome": "success", "summary": {}},
            {"index": 1, "ok": True, "mode": "sequence", "overall_outcome": "success", "summary": {}},
        ],
        "summary": {"message": "Executed 2 mission specs successfully.", "error": None},
    }
    blob = summarize_team_named_mission_specs_result(batch_result)
    print(json.dumps(blob, separators=(",", ":")))
    return 0 if blob.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
