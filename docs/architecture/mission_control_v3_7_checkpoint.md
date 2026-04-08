# Mission Control V3.7 — Automated ROS integration test (checkpoint)

## Purpose

Make the V3.6 **manual** service proof **repeatable**: one pytest module spins the V3.5 handoff **service node** and a **client node** in-process, calls `ProduceSemanticBlockedPassageV35`, and asserts JSON outcomes. **No** launch files, **no** shared-store/facade wiring, **no** real LLM.

---

## Test module

`tests/test_semantic_handoff_service_v37_ros.py`

- **Harness:** `rclpy` + `MultiThreadedExecutor` (server + client nodes).
- **Cases:**
  1. Valid envelope + `use_deterministic_fake_adapter: true` → `ingest_stored` + `belief_id`.
  2. Unknown top-level key → `handoff_request_invalid`.
  3. `SemanticProductionHandoffNodeV35(store=...)` with allowlist excluding `robot1` → `ingest_rejected` / allowlist message.
  4. Late `ingest_now_utc_iso` → TTL-at-ingest `ingest_rejected`.

If generated srv or handoff node imports fail, tests are **skipped** (e.g. interfaces not built).

---

## Commands

**Build (once per workspace):**

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ws
colcon build --packages-select multi_robot_mission_stack_interfaces multi_robot_mission_stack
source install/setup.bash
```

**ROS integration pytest (from repo root containing `tests/`):**

```bash
cd /path/to/HDNS
source /opt/ros/humble/setup.bash
source /path/to/ws/install/setup.bash
python3 -m pytest tests/test_semantic_handoff_service_v37_ros.py -v
```

**Offline semantic tests (unchanged):**

```bash
cd /path/to/HDNS
PYTHONPATH= python3 -m pytest tests/test_semantic_handoff_core_v35.py tests/test_semantic_production_ingest_v34.py -q
```

---

## Related

- [mission_control_v3_6_checkpoint.md](mission_control_v3_6_checkpoint.md)
- [mission_control_v3_5_checkpoint.md](mission_control_v3_5_checkpoint.md)
