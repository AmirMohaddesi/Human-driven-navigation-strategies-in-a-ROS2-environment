#!/usr/bin/env python3
"""
Minimal LangGraph-boundary check: MissionAgentFacade.with_ros() + handle_command (no subprocess).

Prerequisites: workspace sourced; mission_bridge_node + stack running.

  python3 scripts/validate_langgraph_boundary_ros.py
  python3 scripts/validate_langgraph_boundary_ros.py --also-pose --x 1.0 --y 2.0 --yaw 0.0
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List, Tuple

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.agent import MissionAgentFacade  # noqa: E402


def _emit(obj: Dict[str, Any]) -> None:
    print(json.dumps(obj, separators=(",", ":")))


def _navigate_ok(res: Dict[str, Any]) -> Tuple[bool, str]:
    st = str(res.get("status", "")).lower()
    if st in ("failed", "failure"):
        return False, "navigate_status_failed"
    gid = res.get("goal_id")
    if not gid or not str(gid).strip():
        return False, "missing_goal_id"
    return True, ""


def _query_ok(res: Dict[str, Any]) -> Tuple[bool, str]:
    if res.get("status") != "success":
        return False, "query_status_not_success"
    ns = res.get("nav_status")
    if ns is None or not str(ns).strip():
        return False, "missing_nav_status"
    return True, ""


def _run_nav_query(
    facade: MissionAgentFacade,
    nav_cmd: Dict[str, Any],
    label: str,
) -> Tuple[bool, List[Dict[str, Any]], str]:
    steps: List[Dict[str, Any]] = []
    nav = facade.handle_command(nav_cmd)
    steps.append({"step": f"{label}_navigate", "command": nav_cmd, "result": nav})
    ok, reason = _navigate_ok(nav)
    if not ok:
        return False, steps, reason
    gid = str(nav["goal_id"])
    q_cmd = {
        "type": "query",
        "target": "navigation_state",
        "robot_id": str(nav_cmd.get("robot_id", "")),
        "goal_id": gid,
    }
    st = facade.handle_command(q_cmd)
    steps.append({"step": f"{label}_query_state", "command": q_cmd, "result": st})
    ok2, reason2 = _query_ok(st)
    if not ok2:
        return False, steps, reason2
    return True, steps, ""


def main() -> int:
    p = argparse.ArgumentParser(description="Validate MissionAgentFacade ROS boundary (in-process).")
    p.add_argument("--robot-id", default="robot1")
    p.add_argument("--location-name", default="base")
    p.add_argument("--bridge-node", default="mission_bridge_node")
    p.add_argument("--also-pose", action="store_true", help="After named path, run navigate-pose + query.")
    p.add_argument("--x", type=float, default=0.0)
    p.add_argument("--y", type=float, default=0.0)
    p.add_argument("--yaw", type=float, default=0.0)
    args = p.parse_args()

    summary: Dict[str, Any] = {
        "boundary": "MissionAgentFacade.with_ros_handle_command",
        "ok": False,
        "steps": [],
    }

    facade: MissionAgentFacade | None = None
    try:
        facade = MissionAgentFacade.with_ros(bridge_node_name=args.bridge_node)
    except Exception as exc:
        summary["error"] = {"phase": "facade_construct", "message": str(exc)}
        _emit(summary)
        return 1

    try:
        nav_named = {
            "type": "navigate",
            "target": "named_location",
            "robot_id": args.robot_id,
            "location_name": args.location_name,
        }
        ok, steps, reason = _run_nav_query(facade, nav_named, "named")
        summary["steps"].extend(steps)
        if not ok:
            summary["error"] = {"phase": "named_location", "reason": reason}
            _emit(summary)
            return 1

        if args.also_pose:
            nav_pose = {
                "type": "navigate",
                "target": "pose",
                "robot_id": args.robot_id,
                "x": args.x,
                "y": args.y,
                "yaw": args.yaw,
            }
            ok2, steps2, reason2 = _run_nav_query(facade, nav_pose, "pose")
            summary["steps"].extend(steps2)
            if not ok2:
                summary["error"] = {"phase": "navigate_pose", "reason": reason2}
                _emit(summary)
                return 1

        summary["ok"] = True
        _emit(summary)
        return 0
    except Exception as exc:
        summary["error"] = {"phase": "unexpected", "message": str(exc)}
        _emit(summary)
        return 1
    finally:
        if facade is not None:
            try:
                facade.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
