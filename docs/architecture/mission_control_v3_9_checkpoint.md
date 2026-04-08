# Mission Control V3.9 — Bounded multi-process / launch proof (checkpoint)

## Strategic question

Can the frozen semantic handoff path and the bounded advisory mission-layer seam survive a **launch-based, multi-process** ROS 2 topology **without** changing semantic meaning, authority boundaries, or policy behavior?

## Purpose

Runtime **topology realism**, not semantic redesign: handoff runs in one process, advisory observation runs in another, linked only by existing DDS and the **frozen V3.0.1** `blocked_passage` JSON transport shape.

## Already proven (V3.8 and earlier)

- Single-process handoff service, single-process shared-store + facade seam, deterministic gates, advisory-only block outcome.

## Newly proven by V3.9

- `ros2 launch multi_robot_mission_stack semantic_handoff_advisory_v39.launch.py` brings up **two nodes** (separate processes under the launch supervisor).
- Handoff service accepts a **wall-aligned** bounded envelope; optional **mirror** publishes the ingested record on `/semantic/blocked_passage_v301` (same encoding as V3.0.1 transport).
- Witness process subscribes, ingests via the **unchanged** transport→store path, and exposes a **narrow** service `QueryAdvisoryNamedNavV39` that runs the **existing** `MissionAgentFacade` + `navigate_to_named_location` advisory check on that local store.
- Automated pytest drives handoff + witness **across** that topology (orchestrator is a third client process).

## Intentionally not proven

- Real LLM on the service path in this topology was **out of scope for V3.9** (fake adapter only). Bounded opt-in real-model smoke on the **same** launch file is documented in [mission_control_v4_0_checkpoint.md](mission_control_v4_0_checkpoint.md).
- Second fact type, store redesign, mission bridge integration.
- Product / operational hardening.

## Topology (minimal)

| Process role | Executable | Notes |
|--------------|------------|--------|
| Handoff + optional mirror | `semantic-production-handoff-v35` | Params: `mirror_ingested_record_to_transport:=true`, shared topic name. |
| Advisory witness | `advisory-seam-witness-v39` | Subscribes to transport JSON; `BlockedPassageSharedStoreRuntime` + `QueryAdvisoryNamedNavV39`. |

**IPC:** `std_msgs/String` JSON on the frozen topic (not a new semantic schema). **Witness-only** srv `QueryAdvisoryNamedNavV39` is test/demo plumbing, not a broadened mission bridge API.

## Commands

**Build (once):**

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ws
colcon build --packages-select multi_robot_mission_stack_interfaces multi_robot_mission_stack
source install/setup.bash
```

**Manual launch:**

```bash
ros2 launch multi_robot_mission_stack semantic_handoff_advisory_v39.launch.py
```

Then, in another terminal (with same workspace sourced), call the handoff service with a valid JSON envelope (`use_deterministic_fake_adapter: true`) and the witness `query_advisory_named_nav_v39` with a matching `decision_now_utc_iso` within TTL.

**Automated proof (repo root):**

```bash
source /opt/ros/humble/setup.bash
source /path/to/ws/install/setup.bash
cd /path/to/HDNS
python3 -m pytest tests/test_semantic_launch_multiprocess_v39_ros.py -v
```

**Offline / unchanged seams:**

```bash
python3 -m pytest tests/test_shared_store_semantic_policy_seam_v38.py -q
```

## Related

- [mission_control_v3_8_checkpoint.md](mission_control_v3_8_checkpoint.md)
- [mission_control_v3_7_checkpoint.md](mission_control_v3_7_checkpoint.md)
