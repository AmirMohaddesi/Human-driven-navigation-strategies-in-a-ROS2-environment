"""argparse CLI for manual mission commands via ``MissionAgentFacade`` only."""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any, Dict

from .mission_agent_facade import MissionAgentFacade


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="mission-agent",
        description="Run mission commands through MissionAgentFacade (mock or ROS).",
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--mock",
        action="store_true",
        help="Use in-process mock client (no ROS).",
    )
    mode.add_argument(
        "--ros",
        action="store_true",
        help="Use ROS mission bridge client.",
    )

    sub = parser.add_subparsers(dest="command", required=True)

    nav = sub.add_parser("navigate", help="Navigate robot to a named location.")
    nav.add_argument("--robot-id", required=True, help="Robot identifier.")
    nav.add_argument("--location-name", required=True, help="Named location.")

    q = sub.add_parser("query-state", help="Query navigation state for a goal.")
    q.add_argument("--robot-id", required=True, help="Robot identifier.")
    q.add_argument("--goal-id", required=True, help="Goal identifier.")

    return parser


def create_facade_from_args(args: argparse.Namespace) -> MissionAgentFacade:
    if args.mock:
        return MissionAgentFacade.with_mock()
    if args.ros:
        return MissionAgentFacade.with_ros()
    raise RuntimeError("internal error: neither --mock nor --ros (parser should prevent this)")


def _exit_code_from_result(result: Dict[str, Any]) -> int:
    status = result.get("status")
    if status == "failed":
        return 1
    if status in ("accepted", "success"):
        return 0
    return 1


def handle_navigate(args: argparse.Namespace) -> int:
    facade: MissionAgentFacade | None = None
    try:
        facade = create_facade_from_args(args)
        cmd = {
            "type": "navigate",
            "target": "named_location",
            "robot_id": args.robot_id,
            "location_name": args.location_name,
        }
        out = facade.handle_command(cmd)
        print(json.dumps(out, separators=(",", ":")))
        return _exit_code_from_result(out)
    except Exception as exc:
        print(f"mission-agent: error: {exc}", file=sys.stderr)
        return 1
    finally:
        if facade is not None:
            try:
                facade.close()
            except Exception:
                pass


def handle_query_state(args: argparse.Namespace) -> int:
    facade: MissionAgentFacade | None = None
    try:
        facade = create_facade_from_args(args)
        cmd = {
            "type": "query",
            "target": "navigation_state",
            "robot_id": args.robot_id,
            "goal_id": args.goal_id,
        }
        out = facade.handle_command(cmd)
        print(json.dumps(out, separators=(",", ":")))
        return _exit_code_from_result(out)
    except Exception as exc:
        print(f"mission-agent: error: {exc}", file=sys.stderr)
        return 1
    finally:
        if facade is not None:
            try:
                facade.close()
            except Exception:
                pass


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.command == "navigate":
        return handle_navigate(args)
    if args.command == "query-state":
        return handle_query_state(args)
    print("mission-agent: internal error: unknown subcommand", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
