from typing import Dict, Any, Tuple

import os

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import yaml

from multi_robot_mission_stack_interfaces.srv import (
    NavigateToPose,
    GetNavigationState,
    CancelNavigation,
    NavigateToNamedLocation,
)

from .nav2_client import Nav2Client


class MissionBridgeNode(Node):
    """Minimal mission-level bridge node."""

    def __init__(self) -> None:
        super().__init__("mission_bridge_node")

        # Hard-coded mapping: robot_id -> namespace
        self._robot_namespaces: Dict[str, str] = {
            "robot1": "robot1_ns",
            "robot2": "robot2_ns",
        }

        # Per-robot Nav2 clients
        self._nav_clients: Dict[str, Nav2Client] = {}

        # Track active goals: (robot_id, goal_id) -> metadata
        self._active_goals: Dict[Tuple[str, str], Dict[str, Any]] = {}

        # Named locations loaded from config (location_name -> {x, y, yaw})
        self._named_locations: Dict[str, Dict[str, float]] = {}
        self._load_named_locations()

        # ROS2 service interfaces (thin wrappers over bridge methods)
        self._navigate_to_pose_srv = self.create_service(
            NavigateToPose,
            "navigate_to_pose",
            self._handle_navigate_to_pose,
        )
        self._get_navigation_state_srv = self.create_service(
            GetNavigationState,
            "get_navigation_state",
            self._handle_get_navigation_state,
        )
        self._cancel_navigation_srv = self.create_service(
            CancelNavigation,
            "cancel_navigation",
            self._handle_cancel_navigation,
        )
        self._navigate_to_named_location_srv = self.create_service(
            NavigateToNamedLocation,
            "navigate_to_named_location",
            self._handle_navigate_to_named_location,
        )

    def _load_named_locations(self) -> None:
        """Load named locations from the installed config YAML file."""
        try:
            share_dir = get_package_share_directory("multi_robot_mission_stack")
            config_path = os.path.join(share_dir, "config", "named_locations.yaml")
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(f"Failed to resolve package share directory: {exc}")
            self._named_locations = {}
            return

        if not os.path.exists(config_path):
            self.get_logger().warn(
                f"Named locations config not found at '{config_path}'. "
                "navigate_to_named_location will report unknown locations."
            )
            self._named_locations = {}
            return

        try:
            with open(config_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(f"Failed to load named locations YAML: {exc}")
            self._named_locations = {}
            return

        locations = data.get("locations")
        if not isinstance(locations, dict):
            self.get_logger().warn(
                "named_locations.yaml missing top-level 'locations' mapping."
            )
            self._named_locations = {}
            return

        parsed: Dict[str, Dict[str, float]] = {}
        for name, entry in locations.items():
            if not isinstance(entry, dict):
                continue
            try:
                x = float(entry["x"])
                y = float(entry["y"])
                yaw = float(entry["yaw"])
            except Exception:
                continue
            parsed[str(name)] = {"x": x, "y": y, "yaw": yaw}

        self._named_locations = parsed

    def _handle_navigate_to_pose(
        self, request: NavigateToPose.Request, response: NavigateToPose.Response
    ) -> NavigateToPose.Response:
        result = self.navigate_to_pose(
            request.robot_id,
            request.x,
            request.y,
            request.yaw,
        )

        response.status = str(result.get("status", ""))
        response.message = str(result.get("message", ""))

        data = result.get("data", {}) or {}
        response.goal_id = str(data.get("goal_id", ""))
        response.nav_status = str(data.get("nav_status", ""))

        return response

    def _handle_get_navigation_state(
        self,
        request: GetNavigationState.Request,
        response: GetNavigationState.Response,
    ) -> GetNavigationState.Response:
        result = self.get_navigation_state(request.robot_id, request.goal_id)

        response.status = str(result.get("status", ""))
        response.message = str(result.get("message", ""))

        data = result.get("data", {}) or {}
        response.nav_status = str(data.get("nav_status", ""))

        return response

    def _handle_cancel_navigation(
        self,
        request: CancelNavigation.Request,
        response: CancelNavigation.Response,
    ) -> CancelNavigation.Response:
        result = self.cancel_navigation(request.robot_id, request.goal_id)

        response.status = str(result.get("status", ""))
        response.message = str(result.get("message", ""))

        data = result.get("data", {}) or {}
        response.nav_status = str(data.get("nav_status", ""))

        return response

    def _handle_navigate_to_named_location(
        self,
        request: NavigateToNamedLocation.Request,
        response: NavigateToNamedLocation.Response,
    ) -> NavigateToNamedLocation.Response:
        result = self.navigate_to_named_location(
            request.robot_id,
            request.location_name,
        )

        response.status = str(result.get("status", ""))
        response.message = str(result.get("message", ""))

        data = result.get("data", {}) or {}
        response.goal_id = str(data.get("goal_id", ""))
        response.nav_status = str(data.get("nav_status", ""))

        return response

    def _get_or_create_client(self, robot_id: str) -> Nav2Client:
        client = self._nav_clients.get(robot_id)
        if client is not None:
            return client

        namespace = self._robot_namespaces[robot_id]
        client = Nav2Client(self, namespace)
        self._nav_clients[robot_id] = client
        return client

    def navigate_to_pose(self, robot_id: str, x: float, y: float, yaw: float) -> Dict[str, Any]:
        """
        Minimal mission-level NavigateToPose wrapper.

        Returns:
            {
              "status": "...",
              "message": "...",
              "data": {
                "robot_id": "...",
                "goal_id": "...",
                "nav_status": "accepted" | "rejected"
              }
            }
        """
        namespace = self._robot_namespaces.get(robot_id)
        if namespace is None:
            return {
                "status": "failure",
                "message": f"Unknown robot_id '{robot_id}'",
                "data": {
                    "robot_id": robot_id,
                    "goal_id": "",
                    "nav_status": "rejected",
                },
            }

        client = self._get_or_create_client(robot_id)
        result = client.send_goal(x, y, yaw)

        status = result.get("status", "failure")
        message = result.get("message", "")
        goal_id = result.get("goal_id", "")

        if status == "in_progress" and goal_id:
            nav_status = "accepted"
            self._active_goals[(robot_id, goal_id)] = {
                "robot_id": robot_id,
                "goal_id": goal_id,
                "namespace": namespace,
                "target_pose": {"x": x, "y": y, "yaw": yaw},
            }
        else:
            nav_status = "rejected"

        return {
            "status": status,
            "message": message,
            "data": {
                "robot_id": robot_id,
                "goal_id": goal_id,
                "nav_status": nav_status,
            },
        }

    def get_navigation_state(self, robot_id: str, goal_id: str) -> Dict[str, Any]:
        """
        Inspect the current state of a known navigation goal.
        """
        if robot_id not in self._robot_namespaces:
            return {
                "status": "failure",
                "message": f"Unknown robot_id '{robot_id}'",
                "data": {
                    "robot_id": robot_id,
                    "goal_id": goal_id,
                    "nav_status": "unknown",
                },
            }

        key = (robot_id, goal_id)
        if key not in self._active_goals:
            return {
                "status": "failure",
                "message": "Goal id not found for this robot",
                "data": {
                    "robot_id": robot_id,
                    "goal_id": goal_id,
                    "nav_status": "unknown",
                },
            }

        client = self._get_or_create_client(robot_id)
        state = client.get_goal_state()
        nav_status = state.get("nav_status", "unknown")

        return {
            "status": "success",
            "message": state.get("message", ""),
            "data": {
                "robot_id": robot_id,
                "goal_id": goal_id,
                "nav_status": nav_status,
            },
        }

    def cancel_navigation(self, robot_id: str, goal_id: str) -> Dict[str, Any]:
        """
        Request cancellation of a goal created by this bridge.
        """
        if robot_id not in self._robot_namespaces:
            return {
                "status": "failure",
                "message": f"Unknown robot_id '{robot_id}'",
                "data": {
                    "robot_id": robot_id,
                    "goal_id": goal_id,
                    "nav_status": "not_found",
                },
            }

        key = (robot_id, goal_id)
        if key not in self._active_goals:
            return {
                "status": "failure",
                "message": "Goal id not found for this robot",
                "data": {
                    "robot_id": robot_id,
                    "goal_id": goal_id,
                    "nav_status": "not_found",
                },
            }

        client = self._get_or_create_client(robot_id)
        cancel_result = client.cancel_active_goal()
        nav_status = cancel_result.get("nav_status", "unknown")

        # Do not clean up registry yet; keep behavior simple.
        return {
            "status": "success",
            "message": cancel_result.get("message", ""),
            "data": {
                "robot_id": robot_id,
                "goal_id": goal_id,
                "nav_status": nav_status,
            },
        }

    def navigate_to_named_location(
        self, robot_id: str, location_name: str
    ) -> Dict[str, Any]:
        """
        Navigate a robot to a named location defined in the config.
        """
        if robot_id not in self._robot_namespaces:
            return {
                "status": "failure",
                "message": f"Unknown robot_id '{robot_id}'",
                "data": {
                    "robot_id": robot_id,
                    "location_name": location_name,
                    "goal_id": "",
                    "nav_status": "rejected",
                },
            }

        loc = self._named_locations.get(location_name)
        if loc is None:
            return {
                "status": "failure",
                "message": f"Unknown location_name '{location_name}'",
                "data": {
                    "robot_id": robot_id,
                    "location_name": location_name,
                    "goal_id": "",
                    "nav_status": "rejected",
                },
            }

        nav_result = self.navigate_to_pose(
            robot_id,
            loc["x"],
            loc["y"],
            loc["yaw"],
        )

        status = nav_result.get("status", "failure")
        message = nav_result.get("message", "")
        data = nav_result.get("data", {}) or {}

        return {
            "status": status,
            "message": message,
            "data": {
                "robot_id": data.get("robot_id", robot_id),
                "location_name": location_name,
                "goal_id": data.get("goal_id", ""),
                "nav_status": data.get("nav_status", "rejected"),
            },
        }


def _manual_test() -> None:
    """Very lightweight manual test for navigation helpers.

    To exercise the ROS2 services directly (assuming this node is running):

        ros2 service call /mission_bridge_node/navigate_to_pose \\
            multi_robot_mission_stack_interfaces/srv/NavigateToPose \\
            "{robot_id: 'robot1', x: 0.5, y: 0.0, yaw: 0.0}"

        # Replace <GOAL_ID> with the value returned by navigate_to_pose:
        ros2 service call /mission_bridge_node/get_navigation_state \\
            multi_robot_mission_stack_interfaces/srv/GetNavigationState \\
            "{robot_id: 'robot1', goal_id: '<GOAL_ID>'}"

        ros2 service call /mission_bridge_node/cancel_navigation \\
            multi_robot_mission_stack_interfaces/srv/CancelNavigation \\
            "{robot_id: 'robot1', goal_id: '<GOAL_ID>'}"

        ros2 service call /mission_bridge_node/navigate_to_named_location \\
            multi_robot_mission_stack_interfaces/srv/NavigateToNamedLocation \\
            "{robot_id: 'robot1', location_name: 'test_goal'}"
    """
    rclpy.init()
    node = MissionBridgeNode()

    # This assumes a running simulation with 'robot1_ns' Nav2 stack up.
    result = node.navigate_to_pose("robot1", 0.5, 0.0, 0.0)
    print("navigate_to_pose result:", result)

    data = result.get("data", {})
    goal_id = data.get("goal_id")
    if data.get("nav_status") == "accepted" and goal_id:
        state = node.get_navigation_state("robot1", goal_id)
        print("get_navigation_state result:", state)

        # Example cancel usage (disabled by default):
        # cancel = node.cancel_navigation("robot1", goal_id)
        # print("cancel_navigation result:", cancel)

    rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    _manual_test()

