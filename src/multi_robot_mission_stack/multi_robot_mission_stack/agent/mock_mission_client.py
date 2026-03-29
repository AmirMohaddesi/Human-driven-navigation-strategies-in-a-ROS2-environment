"""In-memory mission client for local development and tests (no ROS)."""

from __future__ import annotations

from typing import Any, Dict, Set


class MockMissionClient:
    """Deterministic fake client with simple in-memory goal bookkeeping."""

    def __init__(self) -> None:
        self._counter = 0
        self._goals: Dict[str, Dict[str, Any]] = {}
        self._canceled: Set[str] = set()

    def navigate_to_pose(
        self, robot_id: str, x: float, y: float, yaw: float
    ) -> Dict[str, Any]:
        self._counter += 1
        goal_id = f"mock-goal-{self._counter:03d}"
        self._goals[goal_id] = {
            "robot_id": robot_id,
            "kind": "pose",
            "x": x,
            "y": y,
            "yaw": yaw,
        }
        return {
            "status": "accepted",
            "message": "goal submitted",
            "goal_id": goal_id,
            "nav_status": "submitted",
        }

    def navigate_to_named_location(
        self, robot_id: str, location_name: str
    ) -> Dict[str, Any]:
        self._counter += 1
        goal_id = f"mock-goal-{self._counter:03d}"
        self._goals[goal_id] = {
            "robot_id": robot_id,
            "kind": "named",
            "location_name": location_name,
        }
        return {
            "status": "accepted",
            "message": "goal submitted",
            "goal_id": goal_id,
            "nav_status": "submitted",
        }

    def get_navigation_state(self, robot_id: str, goal_id: str) -> Dict[str, Any]:
        if goal_id in self._canceled:
            return {
                "status": "success",
                "message": "goal state retrieved",
                "nav_status": "canceled",
            }
        if goal_id in self._goals:
            return {
                "status": "success",
                "message": "goal state retrieved",
                "nav_status": "in_progress",
            }
        return {
            "status": "failure",
            "message": "unknown goal_id",
            "nav_status": "unknown",
        }

    def cancel_navigation(self, robot_id: str, goal_id: str) -> Dict[str, Any]:
        if goal_id in self._goals:
            self._canceled.add(goal_id)
            return {
                "status": "success",
                "message": "goal canceled",
                "nav_status": "canceled",
            }
        return {
            "status": "failure",
            "message": "unknown goal_id",
            "nav_status": "unknown",
        }

    def close(self) -> None:
        """No resources to release."""
        return None
