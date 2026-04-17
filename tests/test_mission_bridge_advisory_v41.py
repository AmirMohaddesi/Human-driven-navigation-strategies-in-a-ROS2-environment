"""
V4.1 — mission bridge observes frozen V3.0.1 transport ingest and applies the same advisory
named-location gate (no srv/schema change; default bridge behavior unchanged).
"""

from __future__ import annotations

import uuid
from datetime import datetime, timezone

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String

from multi_robot_mission_stack.bridge.mission_bridge_node import MissionBridgeNode
from multi_robot_mission_stack.semantic.blocked_passage_v301 import SCHEMA_VERSION
from multi_robot_mission_stack.transport.blocked_passage_json_v301 import (
    encode_blocked_passage_record_json,
)


def _to_iso_z(dt: datetime) -> str:
    dt = dt.astimezone(timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S") + ".%03dZ" % (dt.microsecond // 1000)


def _valid_record(*, location: str = "base") -> dict:
    now = datetime.now(timezone.utc)
    return {
        "schema_version": SCHEMA_VERSION,
        "belief_id": str(uuid.uuid4()),
        "fact_type": "blocked_passage",
        "source_robot_id": "robot1",
        "timestamp_utc": _to_iso_z(now),
        "confidence": 0.85,
        "location_ref": location,
        "provenance": {
            "sensor_class": "lidar_occlusion",
            "observation_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
        },
        "ttl_sec": 3600.0,
        "verification_status": "unverified",
    }


TOPIC = "/semantic/blocked_passage_v41_test"


@pytest.fixture
def ros_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _spin_both(ex: MultiThreadedExecutor, n: int = 30) -> None:
    for _ in range(n):
        ex.spin_once(timeout_sec=0.05)


def test_v41_default_bridge_no_advisory_transport(ros_init) -> None:
    bridge = MissionBridgeNode()
    try:
        assert bridge._advisory_store is None
    finally:
        bridge.destroy_node()


def test_v41_bridge_blocks_named_location_after_transport_ingest(ros_init) -> None:
    bridge = MissionBridgeNode(
        parameter_overrides=[
            Parameter(
                "advisory_blocked_passage_transport_topic",
                Parameter.Type.STRING,
                TOPIC,
            ),
            Parameter(
                "advisory_blocked_passage_allowed_source_robot_ids",
                Parameter.Type.STRING_ARRAY,
                ["robot1"],
            ),
        ],
    )
    pub_node = rclpy.create_node("v41_pub_" + uuid.uuid4().hex[:8])
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(bridge)
    ex.add_node(pub_node)
    try:
        pub = pub_node.create_publisher(String, TOPIC, 10)
        msg = String()
        msg.data = encode_blocked_passage_record_json(_valid_record(location="base"))
        pub.publish(msg)
        _spin_both(ex, 40)

        out = bridge.navigate_to_named_location("robot1", "base")
        assert out["status"] == "failed"
        assert out["message"] == "navigation target blocked by peer belief"
        data = out.get("data") or {}
        assert data.get("goal_id") == ""
        assert data.get("nav_status") == "unknown"
    finally:
        ex.remove_node(bridge)
        ex.remove_node(pub_node)
        bridge.destroy_node()
        pub_node.destroy_node()


def test_v41_bridge_malformed_blocked_transport_survives_then_valid_ingests(ros_init) -> None:
    """P3 demo Step C: invalid JSON on advisory topic must not kill the bridge process."""
    bridge = MissionBridgeNode(
        parameter_overrides=[
            Parameter(
                "advisory_blocked_passage_transport_topic",
                Parameter.Type.STRING,
                TOPIC,
            ),
            Parameter(
                "advisory_blocked_passage_allowed_source_robot_ids",
                Parameter.Type.STRING_ARRAY,
                ["robot1"],
            ),
        ],
    )
    pub_node = rclpy.create_node("v41_pub_mal_" + uuid.uuid4().hex[:8])
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(bridge)
    ex.add_node(pub_node)
    try:
        pub = pub_node.create_publisher(String, TOPIC, 10)
        bad = String()
        bad.data = "{not-json"
        pub.publish(bad)
        _spin_both(ex, 30)
        msg = String()
        msg.data = encode_blocked_passage_record_json(_valid_record(location="base"))
        pub.publish(msg)
        _spin_both(ex, 40)
        out = bridge.navigate_to_named_location("robot1", "base")
        assert out["status"] == "failed"
        assert out["message"] == "navigation target blocked by peer belief"
    finally:
        ex.remove_node(bridge)
        ex.remove_node(pub_node)
        bridge.destroy_node()
        pub_node.destroy_node()


def test_v41_bridge_different_location_not_blocked_then_nav2_path(ros_init) -> None:
    bridge = MissionBridgeNode(
        parameter_overrides=[
            Parameter(
                "advisory_blocked_passage_transport_topic",
                Parameter.Type.STRING,
                TOPIC,
            ),
            Parameter(
                "advisory_blocked_passage_allowed_source_robot_ids",
                Parameter.Type.STRING_ARRAY,
                ["robot1"],
            ),
        ],
    )
    pub_node = rclpy.create_node("v41_pub2_" + uuid.uuid4().hex[:8])
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(bridge)
    ex.add_node(pub_node)
    try:
        pub = pub_node.create_publisher(String, TOPIC, 10)
        msg = String()
        msg.data = encode_blocked_passage_record_json(_valid_record(location="other_hall"))
        pub.publish(msg)
        _spin_both(ex, 40)

        out = bridge.navigate_to_named_location("robot1", "base")
        assert out["message"] != "navigation target blocked by peer belief"
    finally:
        ex.remove_node(bridge)
        ex.remove_node(pub_node)
        bridge.destroy_node()
        pub_node.destroy_node()
