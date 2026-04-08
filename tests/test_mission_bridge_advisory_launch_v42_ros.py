"""
V4.2 — launch-based proof: live ``mission_bridge_node`` with V4.1 advisory transport enabled.

Orchestrator publishes frozen V3.0.1 JSON on the launch-configured topic, then calls
``NavigateToNamedLocation`` on the bridge service.

Requires ``ros2`` on PATH and built/sourced workspace (generated srv).
"""

from __future__ import annotations

import os
import signal
import shutil
import subprocess
import time
import uuid

import pytest

rclpy = pytest.importorskip("rclpy")

try:
    from multi_robot_mission_stack_interfaces.srv import NavigateToNamedLocation
except ImportError:
    NavigateToNamedLocation = None  # type: ignore[misc,assignment]

from multi_robot_mission_stack.transport.blocked_passage_json_v301 import (
    encode_blocked_passage_record_json,
)
from multi_robot_mission_stack.semantic.blocked_passage_v301 import SCHEMA_VERSION

TOPIC = "/semantic/blocked_passage_v42_launch"

_SKIP = pytest.mark.skipif(
    NavigateToNamedLocation is None,
    reason="Generated srv not importable — build interfaces and source install/setup.bash",
)


def _ros2_exe() -> str:
    r = shutil.which("ros2")
    if not r:
        pytest.skip("ros2 not on PATH (source install/setup.bash)")
    return r


def _to_iso_z(ns: float) -> str:
    from datetime import datetime, timezone

    dt = datetime.fromtimestamp(ns, tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S") + ".%03dZ" % (dt.microsecond // 1000)


def _valid_record(*, location: str) -> dict:
    now = time.time()
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


@pytest.fixture
def ros_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def v42_launch_process():
    ros2 = _ros2_exe()
    proc = subprocess.Popen(
        [
            ros2,
            "launch",
            "multi_robot_mission_stack",
            "mission_bridge_advisory_v42.launch.py",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        env=os.environ.copy(),
        start_new_session=True,
    )
    time.sleep(3.0)
    if proc.poll() is not None:
        err = proc.stderr.read().decode() if proc.stderr else ""
        pytest.fail("launch exited early: %s" % err[:2000])
    yield proc
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except Exception:
        proc.terminate()
    try:
        proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        proc.kill()


def _spin(node, n: int = 25) -> None:
    for _ in range(n):
        rclpy.spin_once(node, timeout_sec=0.05)


@_SKIP
def test_v42_launch_bridge_blocks_named_nav_after_transport(ros_init, v42_launch_process) -> None:
    from std_msgs.msg import String

    node = rclpy.create_node("v42_orch_" + str(int(time.time())))
    try:
        pub = node.create_publisher(String, TOPIC, 10)
        cli = node.create_client(NavigateToNamedLocation, "/navigate_to_named_location")
        assert cli.wait_for_service(timeout_sec=30.0)

        msg = String()
        msg.data = encode_blocked_passage_record_json(_valid_record(location="base"))
        pub.publish(msg)
        _spin(node, 40)

        req = NavigateToNamedLocation.Request()
        req.robot_id = "robot1"
        req.location_name = "base"
        fut = cli.call_async(req)
        while not fut.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        res = fut.result()
        assert res is not None
        assert str(res.status) == "failed"
        assert str(res.message) == "navigation target blocked by peer belief"
        assert str(res.goal_id).strip() == ""
        assert str(res.nav_status) == "unknown"
    finally:
        node.destroy_node()


@_SKIP
def test_v42_launch_bridge_other_location_does_not_block_base(ros_init, v42_launch_process) -> None:
    from std_msgs.msg import String

    node = rclpy.create_node("v42_orch_b_" + str(int(time.time())))
    try:
        pub = node.create_publisher(String, TOPIC, 10)
        cli = node.create_client(NavigateToNamedLocation, "/navigate_to_named_location")
        assert cli.wait_for_service(timeout_sec=30.0)

        msg = String()
        msg.data = encode_blocked_passage_record_json(_valid_record(location="other_hall"))
        pub.publish(msg)
        _spin(node, 40)

        req = NavigateToNamedLocation.Request()
        req.robot_id = "robot1"
        req.location_name = "base"
        fut = cli.call_async(req)
        while not fut.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        res = fut.result()
        assert res is not None
        assert str(res.message) != "navigation target blocked by peer belief"
    finally:
        node.destroy_node()
