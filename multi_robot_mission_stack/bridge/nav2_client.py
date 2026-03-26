from typing import Dict, Any, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose

import math
import uuid


class Nav2Client:
    """Minimal NavigateToPose client wrapper scoped to a single namespace."""

    def __init__(self, node: Node, namespace: str) -> None:
        self._node = node
        action_name = f"/{namespace}/navigate_to_pose"
        self._client = ActionClient(node, NavigateToPose, action_name)
        self._goal_handle: Optional[NavigateToPose.Goal] = None  # type: ignore[assignment]
        self._result_future = None

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    def send_goal(self, x: float, y: float, yaw: float) -> Dict[str, Any]:
        """
        Send a NavigateToPose goal and return a minimal status dict.

        Returns:
            {
              "status": "success" | "failure" | "in_progress",
              "message": str,
              "goal_id": str
            }
        """
        if not self._client.wait_for_server(timeout_sec=5.0):
            return {
                "status": "failure",
                "message": "NavigateToPose action server not available",
                "goal_id": ""
            }

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = float(x)
        pose_stamped.pose.position.y = float(y)
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation = self._yaw_to_quaternion(float(yaw))

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        send_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self._node, send_future)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {
                "status": "failure",
                "message": "NavigateToPose goal rejected",
                "goal_id": ""
            }

        # Store for later state/cancel operations.
        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()

        goal_id = str(uuid.uuid4())

        return {
            "status": "in_progress",
            "message": "NavigateToPose goal accepted",
            "goal_id": goal_id
        }

    def has_active_goal(self) -> bool:
        return self._goal_handle is not None and self._result_future is not None

    def cancel_active_goal(self) -> Dict[str, Any]:
        """
        Attempt to cancel the currently tracked goal.

        Returns a dict with a normalized nav_status:
            { "nav_status": "cancelling" | "cancelled" | "not_cancellable" | "unknown" }
        """
        if not self.has_active_goal():
            return {
                "nav_status": "not_cancellable",
                "message": "No active goal to cancel"
            }

        cancel_future = self._goal_handle.cancel_goal_async()  # type: ignore[union-attr]
        rclpy.spin_until_future_complete(self._node, cancel_future)
        cancel_result = cancel_future.result()

        if cancel_result is None:
            return {
                "nav_status": "unknown",
                "message": "Cancel request returned no result"
            }

        # Nav2 will eventually report a canceled result via the result future.
        return {
            "nav_status": "cancelling",
            "message": "Cancel requested"
        }

    def get_goal_state(self) -> Dict[str, Any]:
        """
        Inspect the current goal state, if any, and return a normalized status.

        Returns:
            { "nav_status": "accepted" | "in_progress" | "succeeded" | "failed" | "cancelled" | "unknown" }
        """
        if not self.has_active_goal():
            return {
                "nav_status": "unknown",
                "message": "No active goal"
            }

        if not self._result_future.done():
            return {
                "nav_status": "in_progress",
                "message": "Goal still in progress"
            }

        result = self._result_future.result()
        if result is None:
            return {
                "nav_status": "unknown",
                "message": "No result for completed goal"
            }

        # result is a NavigateToPose result wrapper with .status and .result.path, etc.
        status_code = getattr(result, "status", None)
        if status_code is None:
            return {
                "nav_status": "unknown",
                "message": "Result missing status code"
            }

        # Map common rclpy action status codes.
        # 0: UNKNOWN, 1: ACCEPTED, 2: EXECUTING, 3: CANCELING, 4: SUCCEEDED, 5: CANCELED, 6: ABORTED
        if status_code == 4:
            nav_status = "succeeded"
        elif status_code == 5:
            nav_status = "cancelled"
        elif status_code == 6:
            nav_status = "failed"
        elif status_code in (1, 2, 3):
            nav_status = "in_progress"
        else:
            nav_status = "unknown"

        return {
            "nav_status": nav_status,
            "message": f"Goal finished with status code {status_code}"
        }


