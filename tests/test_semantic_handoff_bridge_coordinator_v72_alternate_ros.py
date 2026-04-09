"""
V7.2 — launch proof: handoff→mirror→bridge advisory primary + V7.1 alternate navigate.

After valid deterministic handoff for ``base``, a single sequential step ``robot1→base`` with
``alternate_location_name=test_goal`` must show advisory-blocked primary, ``alternate_attempted``,
and a non-None ``alternate_result``; ``result`` is the effective (alternate) navigate outcome.

Control: invalid handoff (no belief) → primary is not advisory-classified → no alternate attempt.

Reuses ``semantic_handoff_mission_bridge_v43.launch.py``. No production changes.

The alternate navigate may fail headlessly (no Nav2); V7.2 proves **one** alternate service path
was invoked, not that Nav2 succeeds.

Requires built workspace, ``source install/setup.bash``, ``ros2`` on PATH.
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
    from multi_robot_mission_stack_interfaces.srv import ProduceSemanticBlockedPassageV35
except ImportError:
    ProduceSemanticBlockedPassageV35 = None  # type: ignore[misc,assignment]

from multi_robot_mission_stack.agent.navigate_failure_classification_v51 import (
    NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE,
)
from multi_robot_mission_stack.coordinator import assign_named_sequence
from multi_robot_mission_stack.semantic.semantic_handoff_core_v35 import (
    OUTCOME_HANDOFF_REQUEST_INVALID,
)
from multi_robot_mission_stack.semantic.semantic_production_ingest_v34 import (
    OUTCOME_INGEST_STORED,
)

_SKIP = pytest.mark.skipif(
    ProduceSemanticBlockedPassageV35 is None,
    reason="Generated srv not importable — build interfaces and source install/setup.bash",
)

_STEP_WITH_ALT = [
    {
        "robot_id": "robot1",
        "location_name": "base",
        "alternate_location_name": "test_goal",
    },
]


def _ros2_exe() -> str:
    r = shutil.which("ros2")
    if not r:
        pytest.skip("ros2 not on PATH (source install/setup.bash)")
    return r


def _to_iso_z(dt: datetime) -> str:
    dt = dt.astimezone(timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S") + ".%03dZ" % (dt.microsecond // 1000)


def _envelope_near_now(*, location: str = "base") -> dict:
    now = datetime.now(timezone.utc)
    asm = now
    ing = now + timedelta(seconds=1)
    return {
        "llm_context": {
            "schema_version": "v3.3.llm_context.1",
            "location_ref": location,
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
def v43_handoff_bridge_launch_process():
    ros2 = _ros2_exe()
    proc = subprocess.Popen(
        [
            ros2,
            "launch",
            "multi_robot_mission_stack",
            "semantic_handoff_mission_bridge_v43.launch.py",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        env=os.environ.copy(),
        start_new_session=True,
    )
    time.sleep(4.0)
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
def test_v72_launch_handoff_advisory_primary_triggers_alternate_navigate(
    ros_init, v43_handoff_bridge_launch_process
) -> None:
    node = rclpy.create_node("v72_alt_" + str(int(time.time())))
    try:
        hcli = node.create_client(
            ProduceSemanticBlockedPassageV35,
            "/produce_semantic_blocked_passage_v35",
        )
        assert hcli.wait_for_service(timeout_sec=30.0)

        hreq = ProduceSemanticBlockedPassageV35.Request()
        hreq.json_request = json.dumps(_envelope_near_now(location="base"), separators=(",", ":"))
        hfut = hcli.call_async(hreq)
        while not hfut.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        hres = hfut.result()
        assert hres is not None
        hand = json.loads(hres.json_response)
        assert hand["outcome"] == OUTCOME_INGEST_STORED

        time.sleep(1.5)

        out = assign_named_sequence(
            _STEP_WITH_ALT,
            per_goal_timeout_sec=5.0,
            poll_interval_sec=0.1,
        )
        assert out["steps_run"] == 1
        st = out["steps"][0]
        assert st["alternate_location_name"] == "test_goal"
        pr = st["primary_result"]
        assert pr.get("navigate_failure_kind") == NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE
        assert st["alternate_attempted"] is True
        assert isinstance(st["alternate_result"], dict)
        assert st["alternate_result"] is not None
        assert st["result"] is st["alternate_result"]
    finally:
        node.destroy_node()


@_SKIP
def test_v72_launch_invalid_handoff_no_alternate_attempt(
    ros_init, v43_handoff_bridge_launch_process
) -> None:
    node = rclpy.create_node("v72_ctrl_" + str(int(time.time())))
    try:
        hcli = node.create_client(
            ProduceSemanticBlockedPassageV35,
            "/produce_semantic_blocked_passage_v35",
        )
        assert hcli.wait_for_service(timeout_sec=30.0)

        hreq = ProduceSemanticBlockedPassageV35.Request()
        hreq.json_request = "{"
        hfut = hcli.call_async(hreq)
        while not hfut.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        hres = hfut.result()
        assert hres is not None
        hand = json.loads(hres.json_response)
        assert hand["outcome"] == OUTCOME_HANDOFF_REQUEST_INVALID

        out = assign_named_sequence(
            _STEP_WITH_ALT,
            per_goal_timeout_sec=5.0,
            poll_interval_sec=0.1,
        )
        assert out["steps_run"] == 1
        st = out["steps"][0]
        assert st["primary_result"].get("navigate_failure_kind") != (
            NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE
        )
        assert st["alternate_attempted"] is False
        assert st["alternate_result"] is None
        assert st["result"] is st["primary_result"]
    finally:
        node.destroy_node()
