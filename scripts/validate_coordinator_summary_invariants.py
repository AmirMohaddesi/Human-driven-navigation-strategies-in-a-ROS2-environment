#!/usr/bin/env python3
"""
Smoke-test summary invariant validators (no ROS).

  python3 scripts/validate_coordinator_summary_invariants.py
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.coordinator import (  # noqa: E402
    validate_team_named_mission_specs_summary,
    validate_team_named_mission_summary,
)


def main() -> int:
    single = {
        "ok": True,
        "mode": "parallel",
        "overall_outcome": "success",
        "mission_state": "completed",
        "total_steps": 1,
        "steps_run": 1,
        "succeeded_count": 1,
        "failed_count": 0,
        "stopped_early": None,
        "continue_on_failure": None,
        "failed_step_indices": [],
        "succeeded_step_indices": [0],
        "first_failed_step_index": None,
        "last_step_index_run": 0,
        "message": "ok",
    }
    batch = {
        "ok": True,
        "overall_outcome": "success",
        "mission_state": "completed",
        "total_specs": 1,
        "specs_run": 1,
        "succeeded_count": 1,
        "failed_count": 0,
        "stopped_early": False,
        "continue_on_batch_failure": False,
        "failed_spec_indices": [],
        "succeeded_spec_indices": [0],
        "first_failed_spec_index": None,
        "last_spec_index_run": 0,
        "message": "ok",
    }
    a = validate_team_named_mission_summary(single)
    b = validate_team_named_mission_specs_summary(batch)
    print(json.dumps({"single": a, "batch": b}, separators=(",", ":")))
    return 0 if a.get("ok") and b.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
