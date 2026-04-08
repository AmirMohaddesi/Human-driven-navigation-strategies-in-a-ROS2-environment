"""
V5.3 — launch runtime proof: live bridge (V4.2 topology) + ``assign_named_navigation``.

After frozen V3.0.1 JSON on the launch-configured advisory topic, the coordinator leg must
surface ``navigate_failure_kind == advisory_blocked_passage`` through the real
``MissionAgentFacade`` / ``MissionClient`` path. Control: block a different location — no
such classification for ``base``.

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

from multi_robot_mission_stack.coordinator import assign_named_navigation
from multi_robot_mission_stack.agent.navigate_failure_classification_v51 import (
    NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE,
)
from multi_robot_mission_stack.semantic.blocked_passage_v301 import SCHEMA_VERSION
from multi_robot_mission_stack.transport.blocked_passage_json_v301 import (
    encode_blocked_passage_record_json,
)

# Must match ``mission_bridge_advisory_v42.launch.py`` ADVISORY_TOPIC.
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
def test_v53_launch_assign_named_blocked_sets_navigate_failure_kind(
    ros_init, v42_launch_process
) -> None:
    from std_msgs.msg import String

    node = rclpy.create_node("v53_orch_" + str(int(time.time())))
    try:
        pub = node.create_publisher(String, TOPIC, 10)
        cli = node.create_client(NavigateToNamedLocation, "/navigate_to_named_location")
        assert cli.wait_for_service(timeout_sec=30.0)

        msg = String()
        msg.data = encode_blocked_passage_record_json(_valid_record(location="base"))
        pub.publish(msg)
        _spin(node, 40)

        out = assign_named_navigation(
            "robot1",
            "base",
            per_goal_timeout_sec=5.0,
            poll_interval_sec=0.1,
        )
        assert out["outcome"] == "failed"
        assert str(out.get("message", "")) == "navigation target blocked by peer belief"
        assert out.get("navigate_failure_kind") == NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE
    finally:
        node.destroy_node()


@_SKIP
def test_v53_launch_assign_named_other_location_no_advisory_classification(
    ros_init, v42_launch_process
) -> None:
    from std_msgs.msg import String

    node = rclpy.create_node("v53_ctrl_" + str(int(time.time())))
    try:
        pub = node.create_publisher(String, TOPIC, 10)
        cli = node.create_client(NavigateToNamedLocation, "/navigate_to_named_location")
        assert cli.wait_for_service(timeout_sec=30.0)

        msg = String()
        msg.data = encode_blocked_passage_record_json(_valid_record(location="other_hall"))
        pub.publish(msg)
        _spin(node, 40)

        out = assign_named_navigation(
            "robot1",
            "base",
            per_goal_timeout_sec=5.0,
            poll_interval_sec=0.1,
        )
        assert str(out.get("message", "")) != "navigation target blocked by peer belief"
        assert "navigate_failure_kind" not in out
    finally:
        node.destroy_node()
