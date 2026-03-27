"""Allowlist policy gate for structured mission requests (no LangGraph, no ROS)."""

from __future__ import annotations

from typing import Any, Dict, Optional

from .policy_config import MissionPolicyConfig


class MissionPolicy:
    """Evaluates whether a request is permitted under ``MissionPolicyConfig``."""

    def __init__(self, config: MissionPolicyConfig) -> None:
        self._config = config

    def evaluate(self, request: Dict[str, Any]) -> Dict[str, Any]:
        if not isinstance(request, dict):
            return self._deny("request must be a dict")

        action_err = self._check_action(request.get("action"))
        if action_err is not None:
            return self._deny(action_err)

        action = str(request["action"]).strip()
        robot_err = self._check_robot_id(request.get("robot_id"))
        if robot_err is not None:
            return self._deny(robot_err)

        robot_id = str(request["robot_id"]).strip()
        if robot_id not in self._config.allowed_robot_ids:
            return self._deny(f"robot_id {robot_id!r} is not permitted")

        if action == "navigate_to_named_location":
            loc_err = self._check_location(request)
            if loc_err is not None:
                return self._deny(loc_err)
            return self._allow()

        if action == "get_navigation_state":
            gid = request.get("goal_id")
            gid_err = self._require_non_empty_string(gid, "goal_id")
            if gid_err is not None:
                return self._deny(gid_err)
            return self._allow()

        return self._deny(f"action {action!r} is not handled")

    def _check_location(self, request: Dict[str, Any]) -> Optional[str]:
        loc = request.get("location_name")
        loc_err = self._require_non_empty_string(loc, "location_name")
        if loc_err is not None:
            return loc_err
        location_name = str(loc).strip()
        if location_name not in self._config.allowed_locations:
            return f"location_name {location_name!r} is not permitted"
        return None

    @staticmethod
    def _allow() -> Dict[str, Any]:
        return {"allowed": True, "message": "request allowed"}

    @staticmethod
    def _deny(reason: str) -> Dict[str, Any]:
        return {"allowed": False, "message": f"policy denied: {reason}"}

    @staticmethod
    def _require_non_empty_string(value: Any, field: str) -> Optional[str]:
        if value is None:
            return f"missing required field {field!r}"
        if not isinstance(value, str) or not value.strip():
            return f"{field} must be a non-empty string"
        return None

    def _check_action(self, action: Any) -> Optional[str]:
        if action is None:
            return "missing required field 'action'"
        if not isinstance(action, str) or not action.strip():
            return "action must be a non-empty string"
        name = action.strip()
        if name not in self._config.allowed_actions:
            return f"action {name!r} is not permitted"
        return None

    def _check_robot_id(self, robot_id: Any) -> Optional[str]:
        return self._require_non_empty_string(robot_id, "robot_id")
