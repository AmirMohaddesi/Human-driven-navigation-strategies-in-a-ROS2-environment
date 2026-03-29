"""In-process poll until navigation reaches a terminal state (via MissionAgentFacade)."""

from __future__ import annotations

import time
from typing import Any, Dict, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from .mission_agent_facade import MissionAgentFacade


def terminal_outcome(payload: Dict[str, Any]) -> Optional[str]:
    """
    If this query-state result is terminal, return a normalized outcome name; else None.
    Matches scripts/wait_for_goal.py semantics.
    """
    nav = str(payload.get("nav_status", "") or "").strip().lower()
    status = str(payload.get("status", "") or "").strip().lower()
    msg = str(payload.get("message", "") or "").lower()

    if nav in ("succeeded", "cancelled", "failed", "rejected", "not_found"):
        return nav

    if status in ("failed", "failure"):
        if "not found" in msg:
            return "not_found"
        return "failed"

    return None


def _query_command(robot_id: str, goal_id: str) -> Dict[str, Any]:
    return {
        "type": "query",
        "target": "navigation_state",
        "robot_id": str(robot_id).strip(),
        "goal_id": str(goal_id).strip(),
    }


def _result_dict(
    outcome: str,
    robot_id: str,
    goal_id: str,
    payload: Dict[str, Any],
    polls: int,
    elapsed_sec: float,
) -> Dict[str, Any]:
    return {
        "outcome": outcome,
        "robot_id": str(robot_id).strip(),
        "goal_id": str(goal_id).strip(),
        "status": str(payload.get("status", "") or ""),
        "nav_status": str(payload.get("nav_status", "") or ""),
        "message": str(payload.get("message", "") or ""),
        "polls": polls,
        "elapsed_sec": round(float(elapsed_sec), 3),
    }


def wait_for_terminal_navigation_state(
    facade: MissionAgentFacade,
    robot_id: str,
    goal_id: str,
    *,
    timeout_sec: float,
    poll_interval_sec: float,
) -> Dict[str, Any]:
    """
    Poll ``facade.handle_command`` with structured query-state commands until a terminal
    outcome or wall-clock timeout. Does not shell out.
    """
    rid = str(robot_id).strip()
    gid = str(goal_id).strip()
    start = time.monotonic()
    deadline = start + float(timeout_sec)
    polls = 0
    last: Dict[str, Any] = {}
    interval = max(0.1, float(poll_interval_sec))

    try:
        while time.monotonic() < deadline:
            polls += 1
            try:
                res = facade.handle_command(_query_command(rid, gid))
            except Exception as exc:
                elapsed = time.monotonic() - start
                return {
                    "outcome": "failed",
                    "robot_id": rid,
                    "goal_id": gid,
                    "status": "failed",
                    "nav_status": "unknown",
                    "message": str(exc),
                    "polls": polls,
                    "elapsed_sec": round(elapsed, 3),
                }

            if not isinstance(res, dict):
                elapsed = time.monotonic() - start
                return _result_dict(
                    "failed",
                    rid,
                    gid,
                    {
                        "status": "failed",
                        "nav_status": "unknown",
                        "message": "non_dict_handle_command_result",
                    },
                    polls,
                    elapsed,
                )

            if res.get("status") == "failed":
                elapsed = time.monotonic() - start
                return _result_dict(
                    "failed",
                    rid,
                    gid,
                    {
                        "status": res.get("status", ""),
                        "nav_status": res.get("nav_status", "unknown"),
                        "message": res.get("message", ""),
                    },
                    polls,
                    elapsed,
                )

            last = res
            oc = terminal_outcome(res)
            elapsed = time.monotonic() - start
            if oc is not None:
                return _result_dict(oc, rid, gid, res, polls, elapsed)

            time.sleep(interval)

        elapsed = time.monotonic() - start
        return _result_dict("timeout", rid, gid, last, polls, elapsed)
    except Exception as exc:
        elapsed = time.monotonic() - start
        return {
            "outcome": "failed",
            "robot_id": rid,
            "goal_id": gid,
            "status": "failed",
            "nav_status": "unknown",
            "message": str(exc),
            "polls": polls,
            "elapsed_sec": round(elapsed, 3),
        }
