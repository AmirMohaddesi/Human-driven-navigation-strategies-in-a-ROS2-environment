"""Translate external command dicts into strict ``MissionGraph`` request dicts."""

from __future__ import annotations

from typing import Any, Dict, Optional


class CommandAdapter:
    """
    Validates external command shape and maps it to internal graph requests.

    Does not apply mission policy, LangGraph, or ROS.
    """

    def adapt(self, command: Any) -> Dict[str, Any]:
        if not isinstance(command, dict):
            return self._fail("command must be a dict")

        type_err = self._require_non_empty_string(command.get("type"), "type")
        if type_err is not None:
            return self._fail(type_err)

        target_err = self._require_non_empty_string(command.get("target"), "target")
        if target_err is not None:
            return self._fail(target_err)

        cmd_type = str(command["type"]).strip()
        target = str(command["target"]).strip()

        if cmd_type == "navigate":
            if target == "named_location":
                return self._adapt_navigate_named_location(command)
            if target == "pose":
                return self._adapt_navigate_pose(command)
            return self._fail(
                f"unsupported navigate target {target!r}; expected 'named_location' or 'pose'"
            )

        if cmd_type == "query":
            if target == "navigation_state":
                return self._adapt_query_navigation_state(command)
            return self._fail(
                f"unsupported query target {target!r}; expected 'navigation_state'"
            )

        if cmd_type == "cancel":
            if target == "navigation":
                return self._adapt_cancel_navigation(command)
            return self._fail(
                f"unsupported cancel target {target!r}; expected 'navigation'"
            )

        return self._fail(f"unknown command type {cmd_type!r}")

    def adapt_and_validate(self, command: Any) -> Dict[str, Any]:
        """
        Full validation path; currently delegates to ``adapt``.

        Reserved for future extra checks (e.g. schema version, correlation ids).
        """
        return self.adapt(command)

    @staticmethod
    def _fail(message: str) -> Dict[str, Any]:
        return {
            "status": "failed",
            "message": message,
            "nav_status": "unknown",
        }

    @staticmethod
    def _require_non_empty_string(value: Any, field: str) -> Optional[str]:
        if value is None:
            return f"missing required field {field!r}"
        if not isinstance(value, str) or not value.strip():
            return f"{field} must be a non-empty string"
        return None

    def _adapt_navigate_named_location(self, command: Dict[str, Any]) -> Dict[str, Any]:
        rid_err = self._require_non_empty_string(command.get("robot_id"), "robot_id")
        if rid_err is not None:
            return self._fail(rid_err)
        loc_err = self._require_non_empty_string(
            command.get("location_name"), "location_name"
        )
        if loc_err is not None:
            return self._fail(loc_err)
        return {
            "action": "navigate_to_named_location",
            "robot_id": str(command["robot_id"]).strip(),
            "location_name": str(command["location_name"]).strip(),
        }

    @staticmethod
    def _is_numeric(value: Any) -> bool:
        if isinstance(value, bool):
            return False
        return isinstance(value, (int, float))

    def _adapt_navigate_pose(self, command: Dict[str, Any]) -> Dict[str, Any]:
        rid_err = self._require_non_empty_string(command.get("robot_id"), "robot_id")
        if rid_err is not None:
            return self._fail(rid_err)
        for field in ("x", "y", "yaw"):
            if command.get(field) is None:
                return self._fail(f"missing required field {field!r}")
            if not self._is_numeric(command.get(field)):
                return self._fail(f"{field} must be numeric")
        return {
            "action": "navigate_to_pose",
            "robot_id": str(command["robot_id"]).strip(),
            "x": float(command["x"]),
            "y": float(command["y"]),
            "yaw": float(command["yaw"]),
        }

    def _adapt_query_navigation_state(self, command: Dict[str, Any]) -> Dict[str, Any]:
        rid_err = self._require_non_empty_string(command.get("robot_id"), "robot_id")
        if rid_err is not None:
            return self._fail(rid_err)
        gid_err = self._require_non_empty_string(command.get("goal_id"), "goal_id")
        if gid_err is not None:
            return self._fail(gid_err)
        return {
            "action": "get_navigation_state",
            "robot_id": str(command["robot_id"]).strip(),
            "goal_id": str(command["goal_id"]).strip(),
        }

    def _adapt_cancel_navigation(self, command: Dict[str, Any]) -> Dict[str, Any]:
        rid_err = self._require_non_empty_string(command.get("robot_id"), "robot_id")
        if rid_err is not None:
            return self._fail(rid_err)
        gid_err = self._require_non_empty_string(command.get("goal_id"), "goal_id")
        if gid_err is not None:
            return self._fail(gid_err)
        return {
            "action": "cancel_navigation",
            "robot_id": str(command["robot_id"]).strip(),
            "goal_id": str(command["goal_id"]).strip(),
        }
