"""ROS 2 service client for mission bridge services (implements ``MissionClientProtocol``)."""

from __future__ import annotations

import uuid
from typing import Any, Dict, Optional, Type, TypeVar

import rclpy

from multi_robot_mission_stack_interfaces.srv import (
    CancelNavigation,
    GetNavigationState,
    NavigateToNamedLocation,
    NavigateToPose,
)

ReqT = TypeVar("ReqT")


class MissionClient:
    """
    Calls ``mission_bridge_node`` services with bounded waits and plain ``dict`` results.

    Creates a dedicated node and lazily caches service clients. Initializes ``rclpy``
    only when ``rclpy.ok()`` is false at construction; ``close()`` shuts down only
    if this instance performed that initialization.
    """

    def __init__(self, bridge_node_name: str = "mission_bridge_node") -> None:
        self._bridge_node_name = bridge_node_name.strip() or "mission_bridge_node"
        self._owns_rclpy = False
        self._closed = False
        self._node: Optional[Any] = None
        self._clients: Dict[str, Any] = {}

        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True

        self._node = rclpy.create_node(f"mission_client_{uuid.uuid4().hex[:8]}")

    def _service_path(self, suffix: str) -> str:
        return f"/{self._bridge_node_name}/{suffix}"

    @staticmethod
    def _fail_goal(message: str) -> Dict[str, Any]:
        return {
            "status": "failed",
            "message": message,
            "nav_status": "unknown",
            "goal_id": None,
        }

    @staticmethod
    def _fail_state(message: str) -> Dict[str, Any]:
        return {
            "status": "failed",
            "message": message,
            "nav_status": "unknown",
        }

    def _call_service(
        self,
        service_name: str,
        srv_type: Type[Any],
        request: ReqT,
        *,
        timeout_sec: float = 3.0,
        include_goal_id: bool,
    ) -> Dict[str, Any]:
        if self._closed or self._node is None:
            msg = "mission client is closed"
            return self._fail_goal(msg) if include_goal_id else self._fail_state(msg)

        if service_name not in self._clients:
            self._clients[service_name] = self._node.create_client(srv_type, service_name)

        client = self._clients[service_name]
        if not client.wait_for_service(timeout_sec=timeout_sec):
            msg = f"service not available: {service_name}"
            return self._fail_goal(msg) if include_goal_id else self._fail_state(msg)

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)

        if not future.done():
            msg = f"service call timed out: {service_name}"
            return self._fail_goal(msg) if include_goal_id else self._fail_state(msg)

        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - transport edge cases
            msg = f"service call failed: {service_name}: {exc}"
            return self._fail_goal(msg) if include_goal_id else self._fail_state(msg)

        if response is None:
            msg = f"empty service response: {service_name}"
            return self._fail_goal(msg) if include_goal_id else self._fail_state(msg)

        if include_goal_id:
            return {
                "status": str(getattr(response, "status", "") or ""),
                "message": str(getattr(response, "message", "") or ""),
                "nav_status": str(getattr(response, "nav_status", "") or ""),
                "goal_id": str(getattr(response, "goal_id", "") or ""),
            }
        return {
            "status": str(getattr(response, "status", "") or ""),
            "message": str(getattr(response, "message", "") or ""),
            "nav_status": str(getattr(response, "nav_status", "") or ""),
        }

    def navigate_to_pose(self, robot_id: str, x: float, y: float, yaw: float) -> dict:
        req = NavigateToPose.Request()
        req.robot_id = robot_id
        req.x = float(x)
        req.y = float(y)
        req.yaw = float(yaw)
        return self._call_service(
            self._service_path("navigate_to_pose"),
            NavigateToPose,
            req,
            include_goal_id=True,
        )

    def navigate_to_named_location(self, robot_id: str, location_name: str) -> dict:
        req = NavigateToNamedLocation.Request()
        req.robot_id = robot_id
        req.location_name = location_name
        return self._call_service(
            self._service_path("navigate_to_named_location"),
            NavigateToNamedLocation,
            req,
            include_goal_id=True,
        )

    def get_navigation_state(self, robot_id: str, goal_id: str) -> dict:
        req = GetNavigationState.Request()
        req.robot_id = robot_id
        req.goal_id = goal_id
        return self._call_service(
            self._service_path("get_navigation_state"),
            GetNavigationState,
            req,
            include_goal_id=False,
        )

    def cancel_navigation(self, robot_id: str, goal_id: str) -> dict:
        req = CancelNavigation.Request()
        req.robot_id = robot_id
        req.goal_id = goal_id
        return self._call_service(
            self._service_path("cancel_navigation"),
            CancelNavigation,
            req,
            include_goal_id=False,
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._clients.clear()
        except Exception:
            pass
        node = self._node
        self._node = None
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if self._owns_rclpy:
            try:
                rclpy.shutdown()
            except Exception:
                pass
