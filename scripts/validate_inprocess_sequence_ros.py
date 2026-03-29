#!/usr/bin/env python3
"""
Validate in-process sequential named navigation (facade + sequence_utils).

Prerequisites: workspace sourced; mission_bridge_node + stack running.

  python3 scripts/validate_inprocess_sequence_ros.py
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.agent import MissionAgentFacade  # noqa: E402
from multi_robot_mission_stack.agent.sequence_utils import (  # noqa: E402
    run_sequential_named_navigation,
)


def _emit(obj: Dict[str, Any]) -> None:
    print(json.dumps(obj, separators=(",", ":")))


def main() -> int:
    p = argparse.ArgumentParser(description="Validate in-process named mission sequence.")
    p.add_argument("--bridge-node", default="mission_bridge_node")
    p.add_argument("--per-goal-timeout", type=float, default=120.0)
    p.add_argument("--poll-interval", type=float, default=1.0)
    p.add_argument(
        "--continue-on-failure",
        action="store_true",
        help="Forward to sequence_utils (default: stop on first failure).",
    )
    args = p.parse_args()

    steps: List[Dict[str, str]] = [
        {"robot_id": "robot1", "location_name": "base"},
        {"robot_id": "robot1", "location_name": "test_goal"},
    ]

    facade: MissionAgentFacade | None = None
    try:
        facade = MissionAgentFacade.with_ros(bridge_node_name=args.bridge_node)
    except Exception as exc:
        _emit({"ok": False, "error": {"phase": "facade_construct", "message": str(exc)}})
        return 1

    try:
        summary = run_sequential_named_navigation(
            facade,
            steps,
            per_goal_timeout_sec=float(args.per_goal_timeout),
            poll_interval_sec=float(args.poll_interval),
            continue_on_failure=bool(args.continue_on_failure),
        )
        ok = summary.get("overall_outcome") == "success"
        _emit({"ok": ok, "summary": summary})
        return 0 if ok else 1
    except Exception as exc:
        _emit({"ok": False, "error": {"phase": "unexpected", "message": str(exc)}})
        return 1
    finally:
        if facade is not None:
            try:
                facade.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
