"""
V10.2 — live runtime proof: ROS degraded ingest → dispatch → ``navigate_named_result`` V10.1 fields.

Reuses V9.4 degraded transport + bridge; calls ``_handle_navigate_to_named_location`` (service
handler path). Captures ``_mission_log`` while still forwarding to the real logger.
"""

from __future__ import annotations

import uuid
from datetime import datetime, timezone
from typing import Any, Dict, List
from unittest.mock import patch

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String

try:
    from multi_robot_mission_stack_interfaces.srv import NavigateToNamedLocation
except ImportError:
    NavigateToNamedLocation = None  # type: ignore[misc,assignment]

from multi_robot_mission_stack.bridge import mission_bridge_node as mbn
from multi_robot_mission_stack.bridge.mission_bridge_node import MissionBridgeNode
from multi_robot_mission_stack.semantic.degraded_passage_candidate_v85 import (
    DegradedPassageAssembledAccept,
    DegradedPassageAssemblyCandidate,
    assemble_degraded_passage_record_from_candidate,
)
from multi_robot_mission_stack.transport.degraded_passage_json_v87 import (
    encode_degraded_passage_record_json,
)

_SKIP = pytest.mark.skipif(
    NavigateToNamedLocation is None,
    reason="Build multi_robot_mission_stack_interfaces and source install/setup.bash",
)


@pytest.fixture
def ros_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _spin_both(ex: MultiThreadedExecutor, n: int = 50) -> None:
    for _ in range(n):
        ex.spin_once(timeout_sec=0.05)


def _mission_log_capture() -> tuple[List[Dict[str, Any]], Any]:
    rows: List[Dict[str, Any]] = []
    real = mbn._mission_log

    def _wrap(logger: Any, event: str, level: str = "info", **fields: Any) -> None:
        rows.append({"event": event, **fields})
        real(logger, event, level=level, **fields)

    return rows, _wrap


@_SKIP
def test_v102_live_navigate_named_result_includes_degraded_context_after_ros_ingest(
    ros_init,
) -> None:
    topic = "/semantic/degraded_passage_v102_" + uuid.uuid4().hex[:10]
    bridge = MissionBridgeNode(
        parameter_overrides=[
            Parameter(
                "advisory_degraded_passage_transport_topic",
                Parameter.Type.STRING,
                topic,
            ),
            Parameter(
                "advisory_degraded_passage_allowed_source_robot_ids",
                Parameter.Type.STRING_ARRAY,
                ["robot1"],
            ),
            Parameter("wait_for_nav2_action_servers", Parameter.Type.BOOL, False),
        ],
    )
    pub_node = rclpy.create_node("v102_pub_" + uuid.uuid4().hex[:8])
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(bridge)
    ex.add_node(pub_node)
    rows, log_wrap = _mission_log_capture()
    try:
        now = datetime.now(timezone.utc)
        asm = assemble_degraded_passage_record_from_candidate(
            DegradedPassageAssemblyCandidate(
                location_ref="base",
                source_robot_id="robot1",
                confidence=0.7,
                ttl_sec=600.0,
                degradation_class="slow_zone",
                recommended_speed_factor=0.6,
                sensor_class="lidar_occlusion",
            ),
            timestamp_utc=now,
            belief_id="b23e4567-e89b-42d3-a456-426614174bbb",
            observation_id="c23e4567-e89b-42d3-a456-426614174ccc",
        )
        assert isinstance(asm, DegradedPassageAssembledAccept)
        pub = pub_node.create_publisher(String, topic, 10)
        pub.publish(String(data=encode_degraded_passage_record_json(asm.record)))
        _spin_both(ex, 60)
        assert bridge._degraded_advisory_store is not None
        assert len(bridge._degraded_advisory_store) >= 1

        fake_pose = {
            "status": "in_progress",
            "message": "goal submitted",
            "data": {
                "robot_id": "robot1",
                "goal_id": "v102-live-goal-001",
                "nav_status": "accepted",
            },
        }
        req = NavigateToNamedLocation.Request()
        req.robot_id = "robot1"
        req.location_name = "base"
        resp = NavigateToNamedLocation.Response()

        with patch.object(bridge, "navigate_to_pose", return_value=fake_pose), patch.object(
            mbn, "_mission_log", side_effect=log_wrap
        ):
            bridge._handle_navigate_to_named_location(req, resp)

        assert str(resp.goal_id).strip() == "v102-live-goal-001"
        nav_events = [r for r in rows if r.get("event") == "navigate_named_result"]
        assert len(nav_events) == 1
        p = nav_events[0]
        assert p["degraded_advisory_has_active"] is True
        assert p["degraded_advisory_entry_count"] >= 1
        assert "degraded_advisory_belief_ids" in p
        assert p["degraded_advisory_belief_ids"][0] == "b23e4567-e89b-42d3-a456-426614174bbb"
    finally:
        ex.remove_node(bridge)
        ex.remove_node(pub_node)
        bridge.destroy_node()
        pub_node.destroy_node()


@_SKIP
def test_v102_control_no_degraded_log_fields_without_degraded_transport(ros_init) -> None:
    bridge = MissionBridgeNode(
        parameter_overrides=[
            Parameter("wait_for_nav2_action_servers", Parameter.Type.BOOL, False),
        ],
    )
    rows, log_wrap = _mission_log_capture()
    try:
        assert bridge._degraded_advisory_store is None
        fake_pose = {
            "status": "in_progress",
            "message": "goal submitted",
            "data": {
                "robot_id": "robot1",
                "goal_id": "v102-ctrl-goal",
                "nav_status": "accepted",
            },
        }
        req = NavigateToNamedLocation.Request()
        req.robot_id = "robot1"
        req.location_name = "base"
        resp = NavigateToNamedLocation.Response()

        with patch.object(bridge, "navigate_to_pose", return_value=fake_pose), patch.object(
            mbn, "_mission_log", side_effect=log_wrap
        ):
            bridge._handle_navigate_to_named_location(req, resp)

        p = [r for r in rows if r.get("event") == "navigate_named_result"][0]
        assert p.get("goal_id") == "v102-ctrl-goal"
        assert "degraded_advisory_has_active" not in p
        assert "degraded_advisory_entry_count" not in p
    finally:
        bridge.destroy_node()
