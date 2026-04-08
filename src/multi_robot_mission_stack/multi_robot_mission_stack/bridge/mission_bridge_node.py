from typing import Any, Dict, FrozenSet, List, Optional, Tuple

import json
import os
import time
from datetime import datetime, timezone

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import yaml

from multi_robot_mission_stack.semantic.blocked_passage_v301 import (
    BLOCKED_PEER_BELIEF_FAILURE_MESSAGE,
    BlockedPassageBeliefStore,
)
from multi_robot_mission_stack.transport.blocked_passage_json_v301 import (
    ingest_blocked_passage_transport_payload,
)

from multi_robot_mission_stack_interfaces.srv import (
    NavigateToPose,
    GetNavigationState,
    CancelNavigation,
    NavigateToNamedLocation,
)

from .nav2_client import Nav2Client


def _advisory_allowlist_from_params(values: list) -> Optional[FrozenSet[str]]:
    if not values:
        return None
    out = frozenset(str(x).strip() for x in values if str(x).strip())
    return out if out else None


def _mission_log(logger: Any, event: str, level: str = "info", **fields: Any) -> None:
    """Structured single-line JSON for grep/journald (not service responses)."""
    payload = {"component": "mission_bridge", "event": event, **fields}
    line = json.dumps(payload, separators=(",", ":"), default=str)
    if level == "debug":
        logger.debug(line)
    elif level == "warn":
        logger.warning(line)
    else:
        logger.info(line)


class MissionBridgeNode(Node):
    """Minimal mission-level bridge node."""

    def __init__(self, *, parameter_overrides: Optional[List[Parameter]] = None) -> None:
        super().__init__(
            "mission_bridge_node",
            parameter_overrides=parameter_overrides or [],
        )

        # Hard-coded mapping: robot_id -> namespace
        self._robot_namespaces: Dict[str, str] = {
            "robot1": "robot1_ns",
            "robot2": "robot2_ns",
        }

        # Per-robot Nav2 clients
        self._nav_clients: Dict[str, Nav2Client] = {}

        # Track active goals: (robot_id, goal_id) -> metadata
        self._active_goals: Dict[Tuple[str, str], Dict[str, Any]] = {}

        # goal_id -> robot_id (derived on register; cancel path ownership lookup)
        self._goal_id_owner: Dict[str, str] = {}

        # Named locations loaded from config (location_name -> {x, y, yaw})
        self._named_locations: Dict[str, Dict[str, float]] = {}
        self._load_named_locations()

        # V4.1 optional: ingest advisory blocked_passage from frozen V3.0.1 transport; gate named nav.
        self.declare_parameter("advisory_blocked_passage_transport_topic", "")
        self.declare_parameter("advisory_blocked_passage_allowed_source_robot_ids", [""])
        self._advisory_store: Optional[BlockedPassageBeliefStore] = None
        self._setup_advisory_blocked_passage_transport()

        # V4.2+: when false, main() skips the Nav2 action-server wait (headless advisory/test launch).
        self.declare_parameter("wait_for_nav2_action_servers", True)

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

    def _setup_advisory_blocked_passage_transport(self) -> None:
        topic = str(self.get_parameter("advisory_blocked_passage_transport_topic").value or "").strip()
        if not topic:
            return
        raw_allow = list(self.get_parameter("advisory_blocked_passage_allowed_source_robot_ids").value)
        allowed = _advisory_allowlist_from_params(raw_allow)
        self._advisory_store = BlockedPassageBeliefStore(allowed_source_robot_ids=allowed)
        self.create_subscription(String, topic, self._on_advisory_blocked_passage_transport, 10)
        self.get_logger().info(
            "V4.1 advisory blocked_passage ingest enabled on %s (named-location gate active)" % topic
        )

    def _on_advisory_blocked_passage_transport(self, msg: String) -> None:
        assert self._advisory_store is not None
        now = self.get_clock().now()
        ns = int(now.nanoseconds)
        now_utc = datetime.fromtimestamp(ns / 1e9, tz=timezone.utc)
        res = ingest_blocked_passage_transport_payload(
            self._advisory_store, msg.data, now_utc=now_utc
        )
        if res.stored:
            self.get_logger().info("advisory blocked_passage ingested at bridge")
        elif res.duplicate_ignored:
            self.get_logger().info("advisory duplicate belief_id ignored")
        elif res.rejected:
            self.get_logger().warning(
                "advisory blocked_passage ingest rejected: %s" % ("; ".join(res.errors),)
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
            str(request.robot_id).strip(),
            float(request.x),
            float(request.y),
            float(request.yaw),
        )

        response.status = str(result.get("status", ""))
        response.message = str(result.get("message", ""))

        data = result.get("data", {}) or {}
        response.goal_id = str(data.get("goal_id", ""))
        response.nav_status = str(data.get("nav_status", ""))

        _mission_log(
            self.get_logger(),
            "navigate_pose_result",
            robot_id=str(request.robot_id).strip(),
            goal_id=str(response.goal_id),
            status=str(response.status),
            nav_status=str(response.nav_status),
            x=float(request.x),
            y=float(request.y),
            yaw=float(request.yaw),
        )
        return response

    def _handle_get_navigation_state(
        self,
        request: GetNavigationState.Request,
        response: GetNavigationState.Response,
    ) -> GetNavigationState.Response:
        rid, gid = str(request.robot_id).strip(), str(request.goal_id).strip()
        _mission_log(
            self.get_logger(),
            "query_state_request",
            level="debug",
            robot_id=rid,
            goal_id=gid,
        )
        result = self.get_navigation_state(rid, gid)

        response.status = str(result.get("status", ""))
        response.message = str(result.get("message", ""))

        data = result.get("data", {}) or {}
        response.nav_status = str(data.get("nav_status", ""))

        _mission_log(
            self.get_logger(),
            "query_state_result",
            level="debug",
            robot_id=rid,
            goal_id=gid,
            status=str(response.status),
            nav_status=str(response.nav_status),
            message=str(response.message)[:200],
        )
        return response

    def _handle_cancel_navigation(
        self,
        request: CancelNavigation.Request,
        response: CancelNavigation.Response,
    ) -> CancelNavigation.Response:
        rid, cid = str(request.robot_id).strip(), str(request.goal_id).strip()
        _mission_log(
            self.get_logger(),
            "cancel_request",
            robot_id=rid,
            goal_id=cid,
        )
        result = self.cancel_navigation(rid, cid)

        response.status = str(result.get("status", ""))
        response.message = str(result.get("message", ""))

        data = result.get("data", {}) or {}
        response.nav_status = str(data.get("nav_status", ""))

        _mission_log(
            self.get_logger(),
            "cancel_result",
            robot_id=rid,
            goal_id=cid,
            status=str(response.status),
            nav_status=str(response.nav_status),
            message=str(response.message)[:200],
        )
        return response

    def _handle_navigate_to_named_location(
        self,
        request: NavigateToNamedLocation.Request,
        response: NavigateToNamedLocation.Response,
    ) -> NavigateToNamedLocation.Response:
        result = self.navigate_to_named_location(
            str(request.robot_id).strip(),
            str(request.location_name).strip(),
        )

        response.status = str(result.get("status", ""))
        response.message = str(result.get("message", ""))

        data = result.get("data", {}) or {}
        response.goal_id = str(data.get("goal_id", ""))
        response.nav_status = str(data.get("nav_status", ""))

        _mission_log(
            self.get_logger(),
            "navigate_named_result",
            robot_id=str(request.robot_id).strip(),
            location_name=str(request.location_name).strip(),
            goal_id=str(response.goal_id),
            status=str(response.status),
            nav_status=str(response.nav_status),
        )
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
        robot_id = str(robot_id).strip()
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
        raw_gid = result.get("goal_id", "")
        goal_id = str(raw_gid).strip() if raw_gid is not None else ""

        # Register whenever Nav2Client returned a goal id (failure paths use "").
        # Do not gate on status == "in_progress" alone: mismatched type/variant status
        # values would skip registration while still returning goal_id to callers.
        if goal_id:
            nav_status = "accepted"
            self._active_goals[(robot_id, goal_id)] = {
                "robot_id": robot_id,
                "goal_id": goal_id,
                "namespace": namespace,
                "target_pose": {"x": x, "y": y, "yaw": yaw},
            }
            self._goal_id_owner[goal_id] = robot_id
        else:
            nav_status = "rejected"

        if goal_id:
            _mission_log(
                self.get_logger(),
                "goal_submitted",
                robot_id=robot_id,
                goal_id=goal_id,
                nav_status=nav_status,
                status=str(status),
                x=float(x),
                y=float(y),
                yaw=float(yaw),
            )
        else:
            _mission_log(
                self.get_logger(),
                "goal_rejected",
                level="warn",
                robot_id=robot_id,
                nav_status=nav_status,
                status=str(status),
                message=str(message)[:200],
            )

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
        robot_id = str(robot_id).strip()
        goal_id = str(goal_id).strip()
        if not robot_id:
            return {
                "status": "failure",
                "message": "robot_id is required",
                "data": {
                    "robot_id": robot_id,
                    "goal_id": goal_id,
                    "nav_status": "unknown",
                },
            }
        if not goal_id:
            return {
                "status": "failure",
                "message": "goal_id is required",
                "data": {
                    "robot_id": robot_id,
                    "goal_id": goal_id,
                    "nav_status": "unknown",
                },
            }
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
        tracked = key in self._active_goals

        client = self._get_or_create_client(robot_id)
        state = client.get_goal_state()
        nav_status = state.get("nav_status", "unknown")
        message = state.get("message", "")
        if not tracked:
            if message:
                message = f"{message} (goal not tracked in bridge registry)"
            else:
                message = "Goal not tracked in bridge registry; state read from Nav2 client"

        return {
            "status": "success",
            "message": message,
            "data": {
                "robot_id": robot_id,
                "goal_id": goal_id,
                "nav_status": nav_status,
            },
        }

    def _classify_cancel_target(self, robot_id: str, goal_id: str) -> str:
        """
        Cancel-path ownership classification (V2.0.1).

        Preconditions: robot_id is a configured mission id (in _robot_namespaces).

        Returns one of:
            owned_active — (robot_id, goal_id) is in _active_goals
            wrong_robot — goal_id is registered to another robot_id
            unknown_goal — no active pair and not wrong_robot
        """
        key = (robot_id, goal_id)
        if key in self._active_goals:
            return "owned_active"
        owner = self._goal_id_owner.get(goal_id)
        if owner is not None and owner != robot_id:
            return "wrong_robot"
        return "unknown_goal"

    def cancel_navigation(self, robot_id: str, goal_id: str) -> Dict[str, Any]:
        """
        Request cancellation of a goal created by this bridge.
        """
        robot_id = str(robot_id).strip()
        goal_id = str(goal_id).strip()
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

        classification = self._classify_cancel_target(robot_id, goal_id)
        if classification == "wrong_robot":
            return {
                "status": "failure",
                "message": "Goal id belongs to another robot",
                "data": {
                    "robot_id": robot_id,
                    "goal_id": goal_id,
                    "nav_status": "wrong_robot",
                },
            }
        if classification == "unknown_goal":
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
        # V2.0.2: tracked goal already terminal in Nav2 — do not send cancel_goal_async.
        pre_state = client.get_goal_state()
        pre_nav = str(pre_state.get("nav_status", "unknown")).strip().lower()
        if pre_nav in ("succeeded", "cancelled", "failed"):
            return {
                "status": "success",
                "message": "Goal is already complete",
                "data": {
                    "robot_id": robot_id,
                    "goal_id": goal_id,
                    "nav_status": "not_cancellable",
                },
            }

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
        robot_id = str(robot_id).strip()
        location_name = str(location_name).strip()
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

        if self._advisory_store is not None:
            now = self.get_clock().now()
            ns = int(now.nanoseconds)
            now_utc = datetime.fromtimestamp(ns / 1e9, tz=timezone.utc)
            q = self._advisory_store.has_active_blocked_passage(
                location_name, now_utc=now_utc
            )
            if q.has_active:
                _mission_log(
                    self.get_logger(),
                    "navigate_named_advisory_blocked",
                    level="warn",
                    robot_id=robot_id,
                    location_name=location_name,
                    active_belief_ids=list(q.active_belief_ids),
                )
                return {
                    "status": "failed",
                    "message": BLOCKED_PEER_BELIEF_FAILURE_MESSAGE,
                    "data": {
                        "robot_id": robot_id,
                        "location_name": location_name,
                        "goal_id": "",
                        "nav_status": "unknown",
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
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        # Wait for Nav2 action servers outside service callbacks so discovery can progress
        # (blocking wait_for_server inside a service handler starves a single-threaded executor).
        if bool(node.get_parameter("wait_for_nav2_action_servers").value):
            deadline = time.monotonic() + 120.0
            while time.monotonic() < deadline:
                for robot_id in node._robot_namespaces:
                    nav_client = node._get_or_create_client(robot_id)
                    if not nav_client.action_server_ready():
                        break
                else:
                    node.get_logger().info(
                        "Nav2 NavigateToPose action servers ready for all configured robots."
                    )
                    break
                executor.spin_once(timeout_sec=0.1)
            else:
                node.get_logger().warn(
                    "Timed out waiting for Nav2 NavigateToPose action servers; "
                    "navigate services may return 'action server not available' until Nav2 is up."
                )
        else:
            node.get_logger().info(
                "Skipping Nav2 action server wait (wait_for_nav2_action_servers:=false)."
            )

        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    _manual_test()

