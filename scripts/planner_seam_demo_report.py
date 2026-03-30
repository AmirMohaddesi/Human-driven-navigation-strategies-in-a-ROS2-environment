#!/usr/bin/env python3
"""
Layer C planner seam demo: offline report (no ROS).

Uses real intent → v1 spec → contract validation → inspect_team_named_mission_spec_checked.
Replaces execute_team_named_mission_spec_checked with a labeled stub so the report runs in CI.

  python3 scripts/planner_seam_demo_report.py
"""

from __future__ import annotations

import json
import os
import sys
from typing import Any, Dict

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.coordinator import inspect_team_named_mission_spec_checked  # noqa: E402
from multi_robot_mission_stack.planner_seam import (  # noqa: E402
    build_team_named_mission_spec_v1_from_intent,
    run_planner_team_named_mission_checked,
)


def _stub_execute_checked(_spec: Any) -> Dict[str, Any]:
    return {
        "ok": True,
        "version": "v1",
        "mode": "sequence",
        "overall_outcome": "success",
        "execution": {},
        "validation": {"ok": True, "overall_outcome": "success", "message": "", "errors": []},
        "summary": {
            "message": "stub: execute_team_named_mission_spec_checked not called (ROS-free demo)",
            "error": None,
        },
    }


def main() -> int:
    intent: Dict[str, Any] = {
        "mode": "sequence",
        "steps": [
            {"robot_id": "robot1", "location_name": "base"},
            {"robot_id": "robot2", "location_name": "goal"},
        ],
    }

    spec_build = build_team_named_mission_spec_v1_from_intent(intent)
    spec = spec_build.get("spec") if isinstance(spec_build.get("spec"), dict) else {}
    inspection = inspect_team_named_mission_spec_checked(spec) if spec_build.get("ok") else None

    seam = run_planner_team_named_mission_checked(
        intent,
        inspect_checked=inspect_team_named_mission_spec_checked,
        execute_checked=_stub_execute_checked,
    )

    report = {
        "artifact": "planner_seam_demo_report",
        "layers": {
            "A": "robot execution / bridge",
            "B": "frozen coordinator contract (v1)",
            "C": "planner_seam: intent → spec → inspect_checked → execute_checked",
        },
        "sample_intent": intent,
        "spec_build": spec_build,
        "inspection_checked": inspection,
        "full_seam_stub_execute": seam,
        "notes": {
            "execute": "execute path uses stub; swap in execute_team_named_mission_spec_checked for live ROS.",
            "no_replanning": "Seam does not retry, replan, or modify Layer B.",
        },
    }

    print("Planner seam demo report (Layer C, ROS-free)")
    print("===========================================")
    print(f"Spec build ok: {spec_build.get('ok')}")
    print(f"Contract ok: {spec_build.get('contract', {}).get('ok')}")
    if inspection is not None:
        print(f"inspect_checked ok: {inspection.get('ok')}")
        print(f"inspect_checked version: {inspection.get('version')!r}")
    print(f"Full seam (stub execute) ok: {seam.get('ok')}")
    print(f"Seam version: {seam.get('version')!r}")
    print()
    print(json.dumps(report, indent=2, default=str))

    build_ok = spec_build.get("ok") is True
    insp_ok = inspection is not None and inspection.get("ok") is True
    seam_ok = seam.get("ok") is True
    return 0 if build_ok and insp_ok and seam_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
