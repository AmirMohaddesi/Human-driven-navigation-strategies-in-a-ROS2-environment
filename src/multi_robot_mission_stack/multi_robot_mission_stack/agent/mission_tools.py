"""Validated, normalized mission tool API for LangGraph (no ROS, no LangGraph)."""

from __future__ import annotations

from datetime import datetime
from typing import Any, Dict, Optional

from ..semantic.blocked_passage_v301 import (
    BLOCKED_PEER_BELIEF_FAILURE_MESSAGE,
    BlockedPassageBeliefStore,
    make_blocked_by_peer_belief_outcome,
)
from .mission_client_protocol import MissionClientProtocol


class MissionTools:
    """Thin tool layer: validate inputs, call a MissionClientProtocol, normalize dict results."""

    def __init__(
        self,
        client: MissionClientProtocol,
        *,
        blocked_passage_store: Optional[BlockedPassageBeliefStore] = None,
    ) -> None:
        self._client = client
        self._blocked_passage_store = blocked_passage_store

    def navigate_to_pose(
        self, robot_id: str, x: float, y: float, yaw: float
    ) -> Dict[str, Any]:
        err = self._validate_robot_id(robot_id)
        if err:
            return self._invalid_goal_response(err)
        coord_err = self._validate_coordinates(x, y, yaw)
        if coord_err:
            return self._invalid_goal_response(coord_err)
        raw = self._client.navigate_to_pose(
            robot_id.strip(),
            float(x),
            float(y),
            float(yaw),
        )
        return self._normalize_goal_response(raw)

    def navigate_to_named_location(
        self,
        robot_id: str,
        location_name: str,
        *,
        now_utc: Optional[datetime] = None,
    ) -> Dict[str, Any]:
        err = self._validate_robot_id(robot_id)
        if err:
            return self._invalid_goal_response(err)
        loc_err = self._validate_location_name(location_name)
        if loc_err:
            return self._invalid_goal_response(loc_err)
        stripped_loc = location_name.strip()
        if self._blocked_passage_store is not None:
            if now_utc is None:
                return self._invalid_goal_response(
                    "now_utc is required when blocked_passage_store is configured"
                )
            q = self._blocked_passage_store.has_active_blocked_passage(
                stripped_loc,
                now_utc=now_utc,
            )
            if q.has_active:
                return self._blocked_by_peer_belief_response(
                    requested_location_name=stripped_loc,
                    active_belief_ids=q.active_belief_ids,
                )
        raw = self._client.navigate_to_named_location(
            robot_id.strip(),
            stripped_loc,
        )
        return self._normalize_goal_response(raw)

    def get_navigation_state(self, robot_id: str, goal_id: str) -> Dict[str, Any]:
        err = self._validate_robot_id(robot_id)
        if err:
            return self._invalid_state_response(err)
        gid_err = self._validate_goal_id(goal_id)
        if gid_err:
            return self._invalid_state_response(gid_err)
        raw = self._client.get_navigation_state(robot_id.strip(), goal_id.strip())
        return self._normalize_state_response(raw)

    def cancel_navigation(self, robot_id: str, goal_id: str) -> Dict[str, Any]:
        err = self._validate_robot_id(robot_id)
        if err:
            return self._invalid_state_response(err)
        gid_err = self._validate_goal_id(goal_id)
        if gid_err:
            return self._invalid_state_response(gid_err)
        raw = self._client.cancel_navigation(robot_id.strip(), goal_id.strip())
        return self._normalize_state_response(raw)

    @staticmethod
    def _validate_robot_id(robot_id: str) -> Optional[str]:
        if not isinstance(robot_id, str) or not robot_id.strip():
            return "robot_id must be a non-empty string"
        return None

    @staticmethod
    def _validate_goal_id(goal_id: str) -> Optional[str]:
        if not isinstance(goal_id, str) or not goal_id.strip():
            return "goal_id must be a non-empty string"
        return None

    @staticmethod
    def _validate_location_name(location_name: str) -> Optional[str]:
        if not isinstance(location_name, str) or not location_name.strip():
            return "location_name must be a non-empty string"
        return None

    @staticmethod
    def _is_numeric(value: Any) -> bool:
        if isinstance(value, bool):
            return False
        return isinstance(value, (int, float))

    def _validate_coordinates(self, x: Any, y: Any, yaw: Any) -> Optional[str]:
        if not self._is_numeric(x):
            return "x must be numeric"
        if not self._is_numeric(y):
            return "y must be numeric"
        if not self._is_numeric(yaw):
            return "yaw must be numeric"
        return None

    @staticmethod
    def _invalid_goal_response(detail: str) -> Dict[str, Any]:
        return {
            "status": "failed",
            "message": f"invalid input: {detail}",
            "nav_status": "unknown",
            "goal_id": None,
        }

    @staticmethod
    def _blocked_by_peer_belief_response(
        *,
        requested_location_name: str,
        active_belief_ids: tuple[str, ...],
    ) -> Dict[str, Any]:
        out = make_blocked_by_peer_belief_outcome(
            requested_location_name=requested_location_name,
            active_belief_ids=active_belief_ids,
        )
        out.update(
            {
                "status": "failed",
                "message": BLOCKED_PEER_BELIEF_FAILURE_MESSAGE,
                "nav_status": "unknown",
                "goal_id": None,
            }
        )
        return out

    @staticmethod
    def _invalid_state_response(detail: str) -> Dict[str, Any]:
        return {
            "status": "failed",
            "message": f"invalid input: {detail}",
            "nav_status": "unknown",
        }

    @staticmethod
    def _normalize_goal_response(raw: Any) -> Dict[str, Any]:
        data = raw if isinstance(raw, dict) else {}
        status = str(data.get("status", "") or "unknown")
        message = str(data.get("message", "") or "")
        nav_status = str(data.get("nav_status", "") or "unknown")
        gid = data.get("goal_id")
        goal_id: Optional[str]
        if gid is None:
            goal_id = None
        else:
            goal_id = str(gid)
        return {
            "status": status,
            "message": message,
            "nav_status": nav_status,
            "goal_id": goal_id,
        }

    @staticmethod
    def _normalize_state_response(raw: Any) -> Dict[str, Any]:
        data = raw if isinstance(raw, dict) else {}
        status = str(data.get("status", "") or "unknown")
        message = str(data.get("message", "") or "")
        nav_status = str(data.get("nav_status", "") or "unknown")
        return {
            "status": status,
            "message": message,
            "nav_status": nav_status,
        }
