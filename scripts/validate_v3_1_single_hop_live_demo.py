#!/usr/bin/env python3
"""
Bounded V3.1 live demo validator: one semantic hop + one blocked navigate (strict classification).

This script composes robot B in-process (same as ``RobotBBlockedPassageDemoRuntime``): navigation must go
through ``facade.handle_command`` with the shared ``BlockedPassageBeliefStore``. A subprocess
``robot-b-blocked-passage-demo`` plus a raw ``ros2 service call`` to the bridge would **not** hit the
policy hook, so this validator embeds the runtime instead of shelling out to it.

Prerequisites (typical):
  - Workspace sourced (``source install/setup.bash``) so ``rclpy`` and interfaces resolve.
  - ``mission_bridge_node`` running and ``/navigate_to_named_location`` available.

Exit codes:
  0 — success (all strict checks passed)
  1 — demo_failed (environment was ready but blocked outcome or ingest contract failed)
  2 — environment_not_ready (ROS/bridge/topic preconditions not met within timeout)

Classification line (stdout): ``CLASSIFICATION: <success|environment_not_ready|demo_failed>``
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
import uuid
from datetime import datetime, timedelta, timezone
from typing import Optional

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)


def _utc_now() -> datetime:
    """Validator wall clock for live demo (explicit at each decision point in logs)."""
    return datetime.now(timezone.utc)


def _print_classification(kind: str, detail: str = "") -> None:
    print(f"CLASSIFICATION: {kind}")
    if detail:
        print(f"DETAIL: {detail}")


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(
        description="V3.1 bounded single-hop live demo validator (robot B composition in-process).",
    )
    parser.add_argument("--timeout-sec", type=float, default=45.0, help="Overall budget")
    parser.add_argument("--bridge-wait-sec", type=float, default=10.0, help="Wait for navigate service")
    parser.add_argument("--ingest-wait-sec", type=float, default=15.0, help="Wait for active belief after publish")
    parser.add_argument("--bridge-node-name", default="mission_bridge_node")
    parser.add_argument("--location-ref", default="base", help="blocked_passage location_ref and navigate target")
    parser.add_argument("--nav-robot-id", default="robot1", help="robot_id for facade navigate command")
    parser.add_argument("--source-robot-id", default="robot_a", help="blocked_passage source_robot_id (allowlist)")
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String

    from multi_robot_mission_stack.client.mission_client import MissionClient
    from multi_robot_mission_stack.demo.robot_a_blocked_passage_sender_v31 import (
        build_robot_a_blocked_passage_wire,
    )
    from multi_robot_mission_stack.demo.robot_b_blocked_passage_runtime_v31 import (
        RobotBBlockedPassageDemoRuntime,
    )
    from multi_robot_mission_stack.semantic.blocked_passage_v301 import (
        BLOCKED_OUTCOME_SCHEMA_VERSION,
        BLOCKED_OUTCOME_VALUE,
        BLOCKED_REASON_CODE,
    )
    from multi_robot_mission_stack.transport.blocked_passage_json_v301 import TRANSPORT_TOPIC_DEFAULT
    from multi_robot_mission_stack_interfaces.srv import NavigateToNamedLocation

    topic = TRANSPORT_TOPIC_DEFAULT
    t0 = time.monotonic()
    deadline = t0 + float(args.timeout_sec)

    def timed_out() -> bool:
        return time.monotonic() >= deadline

    print("=== V3.1 single-hop live demo validator ===")
    print(f"Launched: in-process RobotBBlockedPassageDemoRuntime + transport subscriber on {topic!r}")
    print("Robot A path: in-process std_msgs/String publish (same wire as robot-a-blocked-passage-demo)")
    print(f"Navigate path: facade.handle_command(..., now_utc=...) -> MissionClient -> bridge")

    if not rclpy.ok():
        try:
            rclpy.init()
        except Exception as exc:
            _print_classification("environment_not_ready", f"rclpy.init failed: {exc}")
            return 2

    client: Optional[MissionClient] = None
    recv_node = None
    pub_node = None
    executor: Optional[SingleThreadedExecutor] = None
    spin_thread: Optional[threading.Thread] = None
    stop_spin = threading.Event()

    try:
        client = MissionClient(bridge_node_name=args.bridge_node_name)
        mnode = getattr(client, "_node", None)
        if mnode is None:
            _print_classification("environment_not_ready", "MissionClient has no ROS node")
            return 2

        cli = mnode.create_client(NavigateToNamedLocation, "/navigate_to_named_location")
        if not cli.wait_for_service(timeout_sec=float(args.bridge_wait_sec)):
            _print_classification(
                "environment_not_ready",
                "/navigate_to_named_location not available within bridge wait",
            )
            return 2

        allowed = frozenset({str(args.source_robot_id).strip()})
        runtime = RobotBBlockedPassageDemoRuntime(
            client,
            allowed_source_robot_ids=allowed,
        )
        recv_node = runtime.create_transport_receiver_node()

        executor = SingleThreadedExecutor()

        def _spin() -> None:
            assert executor is not None
            while rclpy.ok() and not stop_spin.is_set():
                executor.spin_once(timeout_sec=0.05)

        executor.add_node(recv_node)
        spin_thread = threading.Thread(target=_spin, daemon=True)
        spin_thread.start()

        # Warm up DDS / executor
        time.sleep(0.3)
        if timed_out():
            _print_classification("environment_not_ready", "timeout before publish")
            return 2

        decision_ts = _utc_now()
        obs_id = "f47ac10b-58cc-4372-a567-0e02b2c3d479"
        wire = build_robot_a_blocked_passage_wire(
            belief_id=str(uuid.uuid4()),
            source_robot_id=str(args.source_robot_id).strip(),
            location_ref=str(args.location_ref).strip(),
            confidence=0.9,
            timestamp_utc=decision_ts - timedelta(seconds=2),
            ttl_sec=600.0,
            sensor_class="v31_live_validator",
            observation_id=obs_id,
        )

        pub_node = rclpy.create_node("v31_single_hop_validator_pub")
        pub = pub_node.create_publisher(String, topic, 10)
        time.sleep(0.2)
        msg = String()
        msg.data = wire
        pub.publish(msg)
        print(f"Published robot A blocked_passage wire ({len(wire)} bytes) to {topic!r}")

        ingest_deadline = time.monotonic() + float(args.ingest_wait_sec)
        ingested = False
        while time.monotonic() < ingest_deadline and not timed_out():
            now_q = _utc_now()
            q = runtime.store.has_active_blocked_passage(str(args.location_ref).strip(), now_utc=now_q)
            if q.has_active:
                ingested = True
                print(f"Ingest observed: active belief(s) for location_ref={args.location_ref!r} ids={q.active_belief_ids!r}")
                break
            time.sleep(0.05)

        if not ingested:
            _print_classification(
                "environment_not_ready",
                "no active blocked_passage in store within ingest wait (DDS/subscriber/clock?)",
            )
            return 2

        nav_now = _utc_now()
        out = runtime.facade.handle_command(
            {
                "type": "navigate",
                "target": "named_location",
                "robot_id": str(args.nav_robot_id).strip(),
                "location_name": str(args.location_ref).strip(),
            },
            now_utc=nav_now,
        )
        print(f"Navigate result keys: {sorted(out.keys())}")
        print(f"outcome={out.get('outcome')!r} reason_code={out.get('reason_code')!r} status={out.get('status')!r}")

        ok = (
            out.get("outcome") == BLOCKED_OUTCOME_VALUE
            and out.get("schema_version") == BLOCKED_OUTCOME_SCHEMA_VERSION
            and out.get("reason_code") == BLOCKED_REASON_CODE
            and str(out.get("requested_location_name", "")).strip() == str(args.location_ref).strip()
            and out.get("goal_id") is None
        )
        if not ok:
            _print_classification("demo_failed", "navigate did not return strict frozen blocked outcome")
            return 1

        _print_classification("success", "ingest active + navigate blocked with frozen outcome")
        return 0
    except KeyboardInterrupt:
        _print_classification("environment_not_ready", "interrupted")
        return 2
    except Exception as exc:
        _print_classification("environment_not_ready", f"unexpected: {exc!r}")
        return 2
    finally:
        stop_spin.set()
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if pub_node is not None:
            try:
                pub_node.destroy_node()
            except Exception:
                pass
        if recv_node is not None:
            try:
                recv_node.destroy_node()
            except Exception:
                pass
        if client is not None:
            client.close()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
