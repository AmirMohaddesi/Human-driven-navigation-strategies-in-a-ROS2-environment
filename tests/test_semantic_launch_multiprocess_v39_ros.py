"""
V3.9 — launch-based multi-process proof: handoff process + witness process (DDS), dynamic clock envelope.

Requires built workspace, ``source install/setup.bash``, and ``ros2`` on PATH.
Skips when imports or ``ros2`` are unavailable.
"""

from __future__ import annotations

import json
import os
import signal
import shutil
import subprocess
import time
from datetime import datetime, timedelta, timezone

import pytest

rclpy = pytest.importorskip("rclpy")

try:
    from multi_robot_mission_stack_interfaces.srv import (
        ProduceSemanticBlockedPassageV35,
        QueryAdvisoryNamedNavV39,
    )
except ImportError:
    ProduceSemanticBlockedPassageV35 = None  # type: ignore[misc,assignment]
    QueryAdvisoryNamedNavV39 = None  # type: ignore[misc,assignment]

from multi_robot_mission_stack.semantic.blocked_passage_v301 import (
    BLOCKED_OUTCOME_VALUE,
)
from multi_robot_mission_stack.semantic.semantic_handoff_core_v35 import (
    OUTCOME_HANDOFF_REQUEST_INVALID,
)
from multi_robot_mission_stack.semantic.semantic_production_ingest_v34 import (
    OUTCOME_INGEST_STORED,
)

_SKIP = pytest.mark.skipif(
    ProduceSemanticBlockedPassageV35 is None or QueryAdvisoryNamedNavV39 is None,
    reason="Generated srv not importable — build interfaces and source install/setup.bash",
)


def _ros2_exe() -> str:
    r = shutil.which("ros2")
    if not r:
        pytest.skip("ros2 not on PATH (source install/setup.bash)")
    return r


def _to_iso_z(dt: datetime) -> str:
    dt = dt.astimezone(timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S") + ".%03dZ" % (dt.microsecond // 1000)


def _envelope_near_now() -> dict:
    """Wall-aligned envelope so transport ingest at witness uses a still-active TTL."""
    now = datetime.now(timezone.utc)
    asm = now
    ing = now + timedelta(seconds=1)
    return {
        "llm_context": {
            "schema_version": "v3.3.llm_context.1",
            "location_ref": "base",
            "source_robot_id": "robot1",
            "nav_goal_status": "active",
            "stall_duration_sec": 1.0,
            "planner_status": "computing",
            "lidar_occlusion_proxy": True,
            "operator_hint": "",
        },
        "assembly_timestamp_utc_iso": _to_iso_z(asm),
        "ingest_now_utc_iso": _to_iso_z(ing),
        "use_deterministic_fake_adapter": True,
    }


@pytest.fixture
def ros_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def v39_launch_process():
    ros2 = _ros2_exe()
    proc = subprocess.Popen(
        [
            ros2,
            "launch",
            "multi_robot_mission_stack",
            "semantic_handoff_advisory_v39.launch.py",
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


@_SKIP
def test_v39_launch_multiprocess_handoff_mirrors_witness_blocks(ros_init, v39_launch_process) -> None:
    node = rclpy.create_node("v39_orchestrator_" + str(int(time.time())))
    try:
        hcli = node.create_client(
            ProduceSemanticBlockedPassageV35,
            "/produce_semantic_blocked_passage_v35",
        )
        qcli = node.create_client(
            QueryAdvisoryNamedNavV39,
            "/query_advisory_named_nav_v39",
        )
        assert hcli.wait_for_service(timeout_sec=30.0)
        assert qcli.wait_for_service(timeout_sec=30.0)

        env = _envelope_near_now()
        ing_ts = datetime.fromisoformat(
            env["ingest_now_utc_iso"].replace("Z", "+00:00"),
        ).astimezone(timezone.utc)
        decision_iso = _to_iso_z(ing_ts + timedelta(seconds=2))

        hreq = ProduceSemanticBlockedPassageV35.Request()
        hreq.json_request = json.dumps(env, separators=(",", ":"))
        hfut = hcli.call_async(hreq)
        while not hfut.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        hres = hfut.result()
        assert hres is not None
        hand = json.loads(hres.json_response)
        assert hand["outcome"] == OUTCOME_INGEST_STORED

        time.sleep(1.5)

        qreq = QueryAdvisoryNamedNavV39.Request()
        qreq.robot_id = "robot1"
        qreq.location_name = "base"
        qreq.decision_now_utc_iso = decision_iso
        qfut = qcli.call_async(qreq)
        while not qfut.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        qres = qfut.result()
        assert qres is not None and qres.ok
        nav = json.loads(qres.json_result)
        assert nav.get("outcome") == BLOCKED_OUTCOME_VALUE
        assert nav.get("goal_id") is None
    finally:
        node.destroy_node()


@_SKIP
def test_v39_handoff_invalid_witness_nav_accepted(ros_init, v39_launch_process) -> None:
    node = rclpy.create_node("v39_orchestrator_b_" + str(int(time.time())))
    try:
        hcli = node.create_client(
            ProduceSemanticBlockedPassageV35,
            "/produce_semantic_blocked_passage_v35",
        )
        qcli = node.create_client(
            QueryAdvisoryNamedNavV39,
            "/query_advisory_named_nav_v39",
        )
        assert hcli.wait_for_service(timeout_sec=30.0)
        assert qcli.wait_for_service(timeout_sec=30.0)

        hreq = ProduceSemanticBlockedPassageV35.Request()
        hreq.json_request = "{"
        hfut = hcli.call_async(hreq)
        while not hfut.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        hres = hfut.result()
        assert hres is not None
        hand = json.loads(hres.json_response)
        assert hand["outcome"] == OUTCOME_HANDOFF_REQUEST_INVALID

        ing_ts = datetime.now(timezone.utc)
        decision_iso = _to_iso_z(ing_ts)

        qreq = QueryAdvisoryNamedNavV39.Request()
        qreq.robot_id = "robot1"
        qreq.location_name = "base"
        qreq.decision_now_utc_iso = decision_iso
        qfut = qcli.call_async(qreq)
        while not qfut.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        qres = qfut.result()
        assert qres is not None and qres.ok
        nav = json.loads(qres.json_result)
        assert nav.get("status") == "accepted"
        assert nav.get("goal_id")
    finally:
        node.destroy_node()
