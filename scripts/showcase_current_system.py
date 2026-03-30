#!/usr/bin/env python3
"""
Static showcase index: implemented mission platform surfaces (no ROS, no execution).

  python3 scripts/showcase_current_system.py
"""

from __future__ import annotations

import json
import sys

SHOWCASE = {
    "ok": True,
    "overall_outcome": "success",
    "layers": {
        "layer_a": {
            "description": "ROS2 mission execution via bridge and MissionAgentFacade",
            "examples": [
                "multi_robot_mission_stack.bridge.mission_bridge_node",
                "multi_robot_mission_stack.agent.mission_agent_facade.MissionAgentFacade",
                "assign_named_navigation (coordinator entry using facade + wait_for_terminal_navigation_state)",
            ],
            "verification": [
                "tests/test_mission_agent_facade.py",
                "scripts/validate_mission_agent_facade_ros.py",
                "scripts/validate_mission_client_ros.py",
                "scripts/validate_inprocess_sequence_ros.py",
                "scripts/validate_inprocess_parallel_ros.py",
                "scripts/validate_inprocess_wait_ros.py",
            ],
        },
        "layer_b": {
            "description": "Team coordination, mission contract v1, manifest, checked flows",
            "package": "multi_robot_mission_stack.coordinator",
            "single_spec": [
                "inspect_team_named_mission_spec",
                "inspect_team_named_mission_spec_checked",
                "execute_team_named_mission_spec",
                "execute_team_named_mission_spec_checked",
            ],
            "batch_specs": [
                "inspect_team_named_mission_specs",
                "inspect_team_named_mission_specs_checked",
                "execute_team_named_mission_specs",
                "execute_team_named_mission_specs_checked",
            ],
            "modes": ["sequence", "parallel"],
            "manifest_plan": [
                "get_team_named_mission_api_manifest",
                "validate_team_named_mission_api_manifest",
                "resolve_team_named_mission_api_entrypoint",
                "validate_team_named_mission_api_request",
                "plan_team_named_mission_api_call",
            ],
            "contract_validators": [
                "validate_team_named_mission_spec_contract",
                "validate_team_named_mission_specs_contract",
                "validate_team_named_mission_execution_contract",
                "validate_team_named_mission_specs_execution_contract",
            ],
            "invariant_validators": [
                "validate_team_named_mission_inspection",
                "validate_team_named_mission_specs_inspection",
                "validate_team_named_mission_summary",
                "validate_team_named_mission_specs_summary",
            ],
        },
        "layer_c": {
            "description": "Planner seam: intent to v1 spec, inspect_checked then execute_checked",
            "module": "multi_robot_mission_stack.planner_seam",
            "functions": [
                "build_team_named_mission_spec_v1_from_intent",
                "run_planner_team_named_mission_checked",
            ],
            "demo_artifacts": [
                "docs/planner_seam_demo.md",
                "scripts/planner_seam_demo_report.py",
            ],
        },
    },
    "contracts": {
        "mission_contract_v1_doc": "docs/mission_contract_v1.md",
        "script": "scripts/validate_mission_contract_v1.py",
        "payload_version_tag": "v1 on stable Layer B returns (additive)",
    },
    "demos": [
        "scripts/demo_layer_b_golden_path.sh",
        "scripts/planner_seam_demo_report.py",
        "scripts/validate_mission_contract_v1.py",
        "scripts/validate_coordinator_inspect_checked.py",
        "scripts/validate_coordinator_batch_inspect_checked.py",
        "scripts/validate_coordinator_execute_checked_ros.py",
        "scripts/validate_coordinator_execute_batch_checked_ros.py",
    ],
    "docs": [
        "docs/showcase_current_system.md",
        "docs/mission_contract_v1.md",
        "docs/planner_seam_demo.md",
        "docs/handoff/manager_handoff_summary.md",
        "docs/handoff/capability_snapshot.md",
        "docs/handoff/golden_path_demo.md",
        "docs/handoff/non_goals_deferred.md",
    ],
    "summary": {
        "message": "Current system showcase generated successfully.",
        "error": None,
    },
}


def main() -> int:
    print(json.dumps(SHOWCASE, separators=(",", ":")))
    return 0 if SHOWCASE.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
