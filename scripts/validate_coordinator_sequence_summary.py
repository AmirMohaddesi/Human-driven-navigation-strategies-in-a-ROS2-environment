#!/usr/bin/env python3
"""
Smoke-test summarize_sequence_result (no ROS).

  python3 scripts/validate_coordinator_sequence_summary.py
"""

from __future__ import annotations

import json
import os
import sys
from typing import Any, Dict

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.coordinator import summarize_sequence_result  # noqa: E402


def _fake_success_summary() -> Dict[str, Any]:
    return {
        "overall_outcome": "success",
        "total_steps": 2,
        "steps_run": 2,
        "succeeded_count": 2,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_failure": False,
        "steps": [
            {"index": 0, "robot_id": "r1", "location_name": "a", "step_outcome": "succeeded"},
            {"index": 1, "robot_id": "r2", "location_name": "b", "step_outcome": "succeeded"},
        ],
    }


def main() -> int:
    blob = summarize_sequence_result(_fake_success_summary())
    print(json.dumps(blob, separators=(",", ":")))
    return 0 if blob.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
