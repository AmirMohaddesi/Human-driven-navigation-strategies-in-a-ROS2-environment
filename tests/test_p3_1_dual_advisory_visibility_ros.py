"""
P3.1 — launch + passive visibility digest (stdout) for dual advisory String transports.

Requires ``ros2`` on PATH, built/sourced workspace, and ``std_msgs`` for publishers.
"""

from __future__ import annotations

import os
import signal
import shutil
import subprocess
import threading
import time
import uuid

import pytest

rclpy = pytest.importorskip("rclpy")
from std_msgs.msg import String  # noqa: E402

from multi_robot_mission_stack.semantic.blocked_passage_v301 import SCHEMA_VERSION
from multi_robot_mission_stack.semantic.degraded_passage_candidate_v85 import (
    DegradedPassageAssembledAccept,
    DegradedPassageAssemblyCandidate,
    assemble_degraded_passage_record_from_candidate,
)
from multi_robot_mission_stack.transport.blocked_passage_json_v301 import encode_blocked_passage_record_json
from multi_robot_mission_stack.transport.degraded_passage_json_v87 import encode_degraded_passage_record_json

TOPIC_BLOCKED = "/semantic/blocked_passage_p1_1"
TOPIC_DEGRADED = "/semantic/degraded_passage_p1_1"


def _ros2_exe() -> str:
    r = shutil.which("ros2")
    if not r:
        pytest.skip("ros2 not on PATH (source install/setup.bash)")
    return r


def _to_iso_z(ns: float) -> str:
    from datetime import datetime, timezone

    dt = datetime.fromtimestamp(ns, tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S") + ".%03dZ" % (dt.microsecond // 1000)


def _valid_blocked_record(*, location: str) -> dict:
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
def p31_launch_process():
    ros2 = _ros2_exe()
    proc = subprocess.Popen(
        [ros2, "launch", "multi_robot_mission_stack", "p3_1_dual_advisory_visibility.launch.py"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=os.environ.copy(),
        text=True,
        bufsize=1,
        start_new_session=True,
    )
    lines: list[str] = []

    def _reader() -> None:
        assert proc.stdout is not None
        for line in iter(proc.stdout.readline, ""):
            lines.append(line)

    th = threading.Thread(target=_reader, daemon=True)
    th.start()
    time.sleep(3.5)
    if proc.poll() is not None:
        pytest.fail("p3.1 launch exited early: " + "".join(lines)[:4000])
    yield proc, lines
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except Exception:
        proc.terminate()
    try:
        proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        proc.kill()
    th.join(timeout=5.0)


def test_p31_visibility_digest_blocked_degraded_and_malformed(ros_init, p31_launch_process) -> None:
    from datetime import datetime, timezone

    proc, lines = p31_launch_process
    node = rclpy.create_node("p31_pub_" + str(int(time.time())))
    try:
        pub_b = node.create_publisher(String, TOPIC_BLOCKED, 10)
        pub_d = node.create_publisher(String, TOPIC_DEGRADED, 10)
        time.sleep(1.0)

        msg_b = String()
        msg_b.data = encode_blocked_passage_record_json(_valid_blocked_record(location="base"))
        pub_b.publish(msg_b)
        for _ in range(40):
            rclpy.spin_once(node, timeout_sec=0.05)

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
        )
        assert isinstance(asm, DegradedPassageAssembledAccept)
        msg_d = String()
        msg_d.data = encode_degraded_passage_record_json(asm.record)
        pub_d.publish(msg_d)
        for _ in range(50):
            rclpy.spin_once(node, timeout_sec=0.05)

        bad = String()
        bad.data = "{not-json"
        pub_b.publish(bad)
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.05)

        time.sleep(1.0)
    finally:
        node.destroy_node()

    text = "".join(lines)
    assert "[P3.1]" in text
    assert TOPIC_BLOCKED in text
    assert TOPIC_DEGRADED in text
    assert "blocked_passage" in text
    assert "degraded_passage" in text
    assert "UNPARSEABLE_JSON" in text
