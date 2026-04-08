# Mission Control V3.8 — Bounded shared-store / policy seam (checkpoint)

## Purpose

Prove **system relevance** without broadening authority: the **same** in-memory `BlockedPassageBeliefStore` instance is written by the **frozen** V3.5 semantic handoff path (`run_handoff_from_json_request` → V3.4 ingest) and **read** by the **existing** advisory policy on `MissionTools.navigate_to_named_location` via `BlockedPassageSharedStoreRuntime` (facade + graph unchanged).

**Not** proven here: multi-process DDS, ROS service + facade in one launch, real LLM, second fact type, mission redesign.

---

## Seam

- **Write path:** `run_handoff_from_json_request` (deterministic fake adapter), same as V3.5 core tests.
- **Read path:** `BlockedPassageSharedStoreRuntime(..., store=shared_store)` → `MissionTools(..., blocked_passage_store=shared_store)` → `MissionAgentFacade.handle_command(..., now_utc=...)`.

No new manager, registry, or policy semantics—only **composition** with one shared store.

---

## Test module

`tests/test_shared_store_semantic_policy_seam_v38.py` (pure Python, no `rclpy`)

1. Valid handoff → `ingest_stored` → named-location command **blocked**; mock client **not** called.
2. Invalid handoff → store empty → named-location **accepted**; mock client called once.

---

## Commands

**V3.8 local proof (from repo root):**

```bash
cd /path/to/HDNS
python3 -m pytest tests/test_shared_store_semantic_policy_seam_v38.py -v
```

**Optional — still not part of V3.8 claim:**

```bash
python3 -m pytest tests/test_semantic_handoff_service_v37_ros.py -v
# (requires sourced ROS workspace after colcon build)
```

---

## Related

- [mission_control_v3_9_checkpoint.md](mission_control_v3_9_checkpoint.md) — launch-based multi-process handoff + witness
- [mission_control_v3_7_checkpoint.md](mission_control_v3_7_checkpoint.md)
- [mission_control_v3_5_checkpoint.md](mission_control_v3_5_checkpoint.md)
- `multi_robot_mission_stack.agent.blocked_passage_shared_runtime_v301.BlockedPassageSharedStoreRuntime`
