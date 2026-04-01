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

    np = sub.add_parser("navigate-pose", help="Navigate robot to map pose (x, y, yaw).")
    np.add_argument("--robot-id", required=True, help="Robot identifier.")
    np.add_argument("--x", type=float, required=True, help="Goal x (map frame).")
    np.add_argument("--y", type=float, required=True, help="Goal y (map frame).")
    np.add_argument("--yaw", type=float, required=True, help="Goal yaw (radians).")

    q = sub.add_parser("query-state", help="Query navigation state for a goal.")
    q.add_argument("--robot-id", required=True, help="Robot identifier.")
    q.add_argument("--goal-id", required=True, help="Goal identifier.")

    c = sub.add_parser("cancel", help="Cancel navigation for a goal.")
    c.add_argument("--robot-id", required=True, help="Robot identifier.")
    c.add_argument("--goal-id", required=True, help="Goal identifier.")

    return parser


def create_facade_from_args(args: argparse.Namespace) -> MissionAgentFacade:
    if args.mock:
        return MissionAgentFacade.with_mock()
    if args.ros:
        return MissionAgentFacade.with_ros()
    raise RuntimeError("internal error: neither --mock nor --ros (parser should prevent this)")


# Process exit mapping (single choke point). Policy: docs/architecture/mission_control_cli_policy.md
# JSON is the primary contract; exit code is a secondary automation signal.
# Contract-valid JSON is not the same as a successful operation: e.g. status "failure"
# with nav_status wrong_robot / not_found is valid, structured output but exit must be nonzero.
_ZERO_EXIT_STATUSES = frozenset({"accepted", "success", "in_progress"})


def _exit_code_from_result(result: Dict[str, Any]) -> int:
    """Map facade result dict to shell exit code.

    * ``status`` in ``_ZERO_EXIT_STATUSES`` -> 0 (success-shaped JSON for CLI defaults).
    * ``failure``, ``failed``, or any other status -> 1 (operational or transport failure;
      parse JSON for wrong_robot vs not_found vs MissionClient ``failed``).
    """
    status = result.get("status")
    if status in _ZERO_EXIT_STATUSES:
        return 0
    return 1


def handle_navigate_pose(args: argparse.Namespace) -> int:
    facade: MissionAgentFacade | None = None
    try:
        facade = create_facade_from_args(args)
        cmd = {
            "type": "navigate",
            "target": "pose",
            "robot_id": args.robot_id,
            "x": args.x,
            "y": args.y,
            "yaw": args.yaw,
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


def handle_cancel(args: argparse.Namespace) -> int:
    facade: MissionAgentFacade | None = None
    try:
        facade = create_facade_from_args(args)
        cmd = {
            "type": "cancel",
            "target": "navigation",
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
    if args.command == "navigate-pose":
        return handle_navigate_pose(args)
    if args.command == "query-state":
        return handle_query_state(args)
    if args.command == "cancel":
        return handle_cancel(args)
    print("mission-agent: internal error: unknown subcommand", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
