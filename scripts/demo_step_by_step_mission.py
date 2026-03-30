#!/usr/bin/env python3
"""
Step-by-step demo runner for the current mission platform.

This is a tiny wrapper over existing Layer B/C surfaces:

- Single-spec dry-run: planner seam intent → v1 spec → inspect_checked
- Single-spec live (with ROS): same, then execute_checked
- Batch dry-run: frozen Layer B batch inspect_checked
- Batch live (with ROS): frozen Layer B batch execute_checked
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any, Dict, List

from multi_robot_mission_stack.coordinator import (
    execute_team_named_mission_spec_checked,
    execute_team_named_mission_specs_checked,
    inspect_team_named_mission_spec_checked,
    inspect_team_named_mission_specs_checked,
)
from multi_robot_mission_stack.planner_seam import (
    build_team_named_mission_spec_v1_from_intent,
)


SingleIntent = Dict[str, Any]
Spec = Dict[str, Any]
BatchSpecs = List[Spec]


def _print_banner(text: str) -> None:
    print(f"== {text} ==")


def _print_json(obj: Any) -> None:
    print(json.dumps(obj, separators=(",", ":")))


def _pause_if_requested(should_pause: bool) -> None:
    if should_pause:
        try:
            input("Press Enter to continue...")
        except EOFError:
            # Non-interactive environment; just continue.
            pass


def _demo_single_dry_run(pause: bool) -> int:
    intent: SingleIntent = {
        "mode": "sequence",
        "steps": [
            {"robot_id": "robot1", "location_name": "base"},
        ],
        "options": {
            "per_goal_timeout_sec": 60.0,
        },
    }

    _print_banner("Build mission spec")
    build_res = build_team_named_mission_spec_v1_from_intent(intent)
    _print_json(build_res)
    if not bool(build_res.get("ok")):
        _pause_if_requested(pause)
        return 1

    spec: Spec = build_res["spec"]
    _pause_if_requested(pause)

    _print_banner("Inspect mission")
    inspect_res = inspect_team_named_mission_spec_checked(spec)
    _print_json(inspect_res)
    _pause_if_requested(pause)

    ok = bool(inspect_res.get("ok"))
    return 0 if ok else 1


def _demo_single_with_ros(pause: bool) -> int:
    intent: SingleIntent = {
        "mode": "sequence",
        "steps": [
            {"robot_id": "robot1", "location_name": "base"},
        ],
        "options": {
            "per_goal_timeout_sec": 60.0,
        },
    }

    _print_banner("Build mission spec")
    build_res = build_team_named_mission_spec_v1_from_intent(intent)
    _print_json(build_res)
    if not bool(build_res.get("ok")):
        _pause_if_requested(pause)
        return 1

    spec: Spec = build_res["spec"]
    _pause_if_requested(pause)

    _print_banner("Inspect mission")
    inspect_res = inspect_team_named_mission_spec_checked(spec)
    _print_json(inspect_res)
    if not bool(inspect_res.get("ok")):
        _pause_if_requested(pause)
        return 1

    _pause_if_requested(pause)

    _print_banner("Execute mission")
    execute_res = execute_team_named_mission_spec_checked(spec)
    _print_json(execute_res)
    _pause_if_requested(pause)

    ok = bool(execute_res.get("ok"))
    return 0 if ok else 1


def _batch_specs() -> BatchSpecs:
    return [
        {
            "version": "v1",
            "mode": "sequence",
            "steps": [
                {"robot_id": "robot1", "location_name": "base"},
            ],
        },
        {
            "version": "v1",
            "mode": "sequence",
            "steps": [
                {"robot_id": "robot2", "location_name": "test_goal"},
            ],
        },
    ]


def _demo_batch_dry_run(pause: bool) -> int:
    specs = _batch_specs()

    _print_banner("Inspect batch")
    inspect_res = inspect_team_named_mission_specs_checked(specs)
    _print_json(inspect_res)
    _pause_if_requested(pause)

    ok = bool(inspect_res.get("ok"))
    return 0 if ok else 1


def _demo_batch_with_ros(pause: bool) -> int:
    specs = _batch_specs()

    _print_banner("Inspect batch")
    inspect_res = inspect_team_named_mission_specs_checked(specs)
    _print_json(inspect_res)
    if not bool(inspect_res.get("ok")):
        _pause_if_requested(pause)
        return 1

    _pause_if_requested(pause)

    _print_banner("Execute batch")
    execute_res = execute_team_named_mission_specs_checked(specs)
    _print_json(execute_res)
    _pause_if_requested(pause)

    ok = bool(execute_res.get("ok"))
    return 0 if ok else 1


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step-by-step mission demo over existing Layer B/C surfaces.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Use offline path only (inspect_checked, no ROS execution).",
    )
    parser.add_argument(
        "--with-ros",
        action="store_true",
        help="Include live execute_checked calls (requires ROS stack running).",
    )
    parser.add_argument(
        "--pause",
        action="store_true",
        help='Pause after each phase with "Press Enter to continue...".',
    )
    parser.add_argument(
        "--batch",
        action="store_true",
        help="Use a tiny hardcoded batch example instead of a single spec.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)

    # Default: single-spec dry-run demo.
    dry_run = True
    with_ros = False
    if args.dry_run:
        dry_run = True
        with_ros = False
    elif args.with_ros:
        dry_run = False
        with_ros = True

    pause = bool(args.pause)
    use_batch = bool(args.batch)

    if use_batch:
        if dry_run:
            return _demo_batch_dry_run(pause)
        return _demo_batch_with_ros(pause)

    if dry_run:
        return _demo_single_dry_run(pause)
    return _demo_single_with_ros(pause)


if __name__ == "__main__":
    raise SystemExit(main())

