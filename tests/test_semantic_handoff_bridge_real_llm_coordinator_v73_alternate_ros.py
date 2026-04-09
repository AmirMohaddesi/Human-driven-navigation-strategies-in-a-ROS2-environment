"""
V7.3 — real-adapter branch on V4.3 launch + coordinator V7.1 alternate-fallback smoke.

- **Default:** launch subprocess strips OpenAI env, ``use_deterministic_fake_adapter: false`` →
  handoff ``adapter_failure`` → no mirror → single step with ``alternate_location_name`` does
  **not** attempt alternate (primary not advisory).
- **Opt-in:** ``RUN_V7_3_LLM_SMOKE=1`` and ``OPENAI_API_KEY`` inherited by launch → ingest, mirror,
  advisory primary → ``alternate_attempted`` and alternate navigate (provider/network dependent).

Same graph as V7.2 / V6.3; no production code changes.
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
from multi_robot_mission_stack.semantic.llm_candidate_eval_v33 import OUTCOME_ADAPTER_FAILURE
from multi_robot_mission_stack.semantic.llm_real_adapter_v33 import ADAPTER_PROVIDER_ERROR
from multi_robot_mission_stack.semantic.semantic_production_ingest_v34 import OUTCOME_INGEST_STORED

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


def _envelope_near_now(*, use_fake: bool, location: str = "base") -> dict:
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
        "use_deterministic_fake_adapter": use_fake,
    }


@pytest.fixture
def ros_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def v43_launch_no_llm_secrets():
    ros2 = _ros2_exe()
    env = os.environ.copy()
    env.pop("OPENAI_API_KEY", None)
    env.pop("OPENAI_BASE_URL", None)
    proc = subprocess.Popen(
        [
            ros2,
            "launch",
            "multi_robot_mission_stack",
            "semantic_handoff_mission_bridge_v43.launch.py",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        env=env,
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


@pytest.fixture
def v43_launch_with_optional_llm_env():
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
def test_v73_e2e_real_adapter_missing_key_no_alternate_attempt(
    ros_init, v43_launch_no_llm_secrets
) -> None:
    node = rclpy.create_node("v73_fail_" + str(int(time.time())))
    try:
        hcli = node.create_client(
            ProduceSemanticBlockedPassageV35,
            "/produce_semantic_blocked_passage_v35",
        )
        assert hcli.wait_for_service(timeout_sec=30.0)

        hreq = ProduceSemanticBlockedPassageV35.Request()
        hreq.json_request = json.dumps(
            _envelope_near_now(use_fake=False), separators=(",", ":")
        )
        hfut = hcli.call_async(hreq)
        while not hfut.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        hres = hfut.result()
        assert hres is not None
        hand = json.loads(hres.json_response)
        assert hand["outcome"] == OUTCOME_ADAPTER_FAILURE
        assert hand.get("adapter_reason") == ADAPTER_PROVIDER_ERROR
        assert "missing api key" in (hand.get("adapter_detail") or "").lower()

        out = assign_named_sequence(
            _STEP_WITH_ALT,
            per_goal_timeout_sec=5.0,
            poll_interval_sec=0.1,
        )
        assert out["steps_run"] == 1
        st = out["steps"][0]
        assert st["alternate_attempted"] is False
        assert st["alternate_result"] is None
        assert st["result"] is st["primary_result"]
        assert st["primary_result"].get("navigate_failure_kind") != (
            NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE
        )
    finally:
        node.destroy_node()


_SKIP_REAL = pytest.mark.skipif(
    not os.environ.get("OPENAI_API_KEY", "").strip()
    or os.environ.get("RUN_V7_3_LLM_SMOKE", "").strip() != "1",
    reason="Set RUN_V7_3_LLM_SMOKE=1 and OPENAI_API_KEY for opt-in real-model alternate smoke.",
)


@_SKIP
@_SKIP_REAL
def test_v73_e2e_real_llm_handoff_alternate_fallback(
    ros_init, v43_launch_with_optional_llm_env
) -> None:
    node = rclpy.create_node("v73_ok_" + str(int(time.time())))
    try:
        hcli = node.create_client(
            ProduceSemanticBlockedPassageV35,
            "/produce_semantic_blocked_passage_v35",
        )
        assert hcli.wait_for_service(timeout_sec=30.0)

        hreq = ProduceSemanticBlockedPassageV35.Request()
        hreq.json_request = json.dumps(
            _envelope_near_now(use_fake=False), separators=(",", ":")
        )
        hfut = hcli.call_async(hreq)
        while not hfut.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        hres = hfut.result()
        assert hres is not None
        hand = json.loads(hres.json_response)
        assert hand["outcome"] == OUTCOME_INGEST_STORED, hand

        time.sleep(2.0)

        out = assign_named_sequence(
            _STEP_WITH_ALT,
            per_goal_timeout_sec=5.0,
            poll_interval_sec=0.1,
        )
        assert out["steps_run"] == 1
        st = out["steps"][0]
        assert st["primary_result"].get("navigate_failure_kind") == (
            NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE
        )
        assert st["alternate_attempted"] is True
        assert isinstance(st["alternate_result"], dict)
        assert st["result"] is st["alternate_result"]
    finally:
        node.destroy_node()
