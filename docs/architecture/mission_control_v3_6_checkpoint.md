# Mission Control V3.6 — Bounded ROS service runtime proof (checkpoint)

## Purpose

Prove in a **built ROS 2 workspace** that the V3.5 **`ProduceSemanticBlockedPassageV35`** service and **`semantic-production-handoff-v35`** node run, and that **bounded JSON requests** return the same structured outcomes as the offline core through the **unchanged** V3.4 path.

This is **runtime evidence**, not production readiness, launch integration, or shared-store/policy wiring.

---

## Build

From workspace root (parent of `src/`):

```bash
source /opt/ros/humble/setup.bash   # or $ROS_DISTRO
colcon build --packages-select multi_robot_mission_stack_interfaces multi_robot_mission_stack
source install/setup.bash
```

---

## Run node

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run multi_robot_mission_stack semantic-production-handoff-v35
```

Optional allowlist proof (excludes `robot1`):

```bash
ros2 run multi_robot_mission_stack semantic-production-handoff-v35 --ros-args \
  --params-file /path/to/HDNS/scripts/params_semantic_handoff_v35_allowlist_other_only.yaml
```

---

## Service call pattern

Use `ros2 service call /produce_semantic_blocked_passage_v35 multi_robot_mission_stack_interfaces/srv/ProduceSemanticBlockedPassageV35` with a JSON argument whose only field is `json_request` (stringified inner JSON). Generating the shell argument with a short Python one-liner avoids escaping errors.

---

## Recorded runtime cases (Humble, 2026-04-08)

**1. Valid + deterministic fake adapter → `ingest_stored`**

- Inner JSON includes `use_deterministic_fake_adapter: true`, explicit `assembly_timestamp_utc_iso` / `ingest_now_utc_iso`, full allowlisted `llm_context` with `source_robot_id: "robot1"`.
- **Response (example):** `{"outcome":"ingest_stored","belief_id":"<uuid-v4>"}`

**2. Invalid envelope → `handoff_request_invalid`**

- Inner JSON includes extra key `extra_key`.
- **Response:** `{"outcome":"handoff_request_invalid","detail":"unknown request keys: ['extra_key']"}`

**3. Allowlist reject → `ingest_rejected`**

- Node started with `allowed_source_robot_ids: ["other_robot"]` via params file; same `llm_context` as (1) with `source_robot_id: "robot1"`.
- **Response (example):** `{"outcome":"ingest_rejected","ingest_errors":["source_robot_id not in allowlist: 'robot1'"],"belief_id":"<uuid-v4>"}`

**4. TTL-at-ingest reject → `ingest_rejected`**

- Same as (1) but `ingest_now_utc_iso` set late (e.g. `2026-04-02T22:00:00.000Z` vs assembly `2026-04-02T18:30:00.000Z` with fake `ttl_sec` 3600).
- **Response:** `ingest_errors` contains `record already expired at ingest time (TTL + skew)`.

**Deferred in this milestone:** `ingest_duplicate_ignored` via service (would require same `belief_id` as a prior ingest; not exposed in the V3.5 JSON envelope without scope change). Launch files, automated `rclpy` integration tests, real LLM.

---

## Minimal code fix for runtime (V3.6)

`declare_parameter("allowed_source_robot_ids", [])` in rclpy infers **BYTE_ARRAY** for `[]`, so YAML **string array** overrides fail. Default was changed to `[""]` so the parameter type is **STRING_ARRAY**; `_allowlist_from_params` still yields **no allowlist** when all entries strip empty. **Only** `semantic_production_handoff_node_v35.py` was changed.

---

## Related

- [mission_control_v3_5_checkpoint.md](mission_control_v3_5_checkpoint.md)
- [mission_control_semantic_checkpoint_through_v34.md](mission_control_semantic_checkpoint_through_v34.md)
