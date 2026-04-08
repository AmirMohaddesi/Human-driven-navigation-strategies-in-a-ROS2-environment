# Mission Control V4.1 — Bounded mission-bridge advisory seam (checkpoint)

## Strategic question

Can the **existing mission bridge** observe the **already-frozen** advisory `blocked_passage` path (same transport ingest and same blocked message semantics) through **one** optional seam—without giving semantic/LLM paths execution authority and without redesigning bridge services, schemas, or Nav2 wiring?

## Purpose

Connect **mission_bridge_node** to the same V3.0.1 JSON belief path used elsewhere: optional subscribe → `BlockedPassageBeliefStore.ingest` → advisory gate on **`navigate_to_named_location` only**.

## Audit summary

- **MissionClient** / **MissionTools** already gate named navigation when a store is injected (V3.8); the **bridge node** previously had **no** belief visibility.
- **Smallest seam:** optional parameters on `MissionBridgeNode` (default **off**): transport topic + source allowlist → reuse `ingest_blocked_passage_transport_payload` → `has_active_blocked_passage` before resolving named coordinates. **No** `.srv` changes; clock for TTL/active checks is the bridge **ROS clock** at request/receive time (same pattern as `BlockedPassageTransportReceiverNode`).

## Already proven (before V4.1)

- Semantic production, handoff, launch+witness, MissionTools/facade advisory gate, V3.0.1 transport encode/decode/ingest.

## Newly proven by V4.1

- With advisory params enabled, a record on the transport topic is ingested at the bridge and **`navigate_to_named_location`** returns `status=failed`, `message=navigation target blocked by peer belief`, empty `goal_id`, `nav_status=unknown`—aligned with `MissionTools` advisory wording.
- Default bridge (empty topic) has **no** advisory store or subscriber.

## Intentionally not proven

- Pose-level advisory at the bridge, mission graph changes, real LLM, multi-process bridge+semantic launch (single-process test here; topology can mirror V3.9 pub to topic).

## Parameters

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `advisory_blocked_passage_transport_topic` | `""` | If non-empty, subscribe and ingest on this topic. |
| `advisory_blocked_passage_allowed_source_robot_ids` | `[""]` | Same allowlist convention as other nodes (all-empty → no allowlist). |

## Commands

**Tests (requires ROS 2 Python, e.g. Humble sourced):**

```bash
source /opt/ros/humble/setup.bash
cd /path/to/HDNS
python3 -m pytest tests/test_mission_bridge_advisory_v41.py -v
```

If `rclpy` is not importable, the module is skipped (same class of environment as other ROS tests).

## Related

- [mission_control_v4_0_checkpoint.md](mission_control_v4_0_checkpoint.md)
- [mission_control_v3_9_checkpoint.md](mission_control_v3_9_checkpoint.md)
