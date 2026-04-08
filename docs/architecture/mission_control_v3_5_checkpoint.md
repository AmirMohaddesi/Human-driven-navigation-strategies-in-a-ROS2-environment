# Mission Control V3.5 — Bounded ROS handoff for semantic production (checkpoint)

## Purpose

Prove **one narrow ROS-facing seam** that exposes the **unchanged** V3.4 path (`produce_and_ingest_blocked_passage_v34`) without redesigning V3.0.1 transport, store semantics, or policy.

**Strategic question answered (local proof):** a JSON service request can trigger the frozen gate stack and, on success, admit through **existing** `BlockedPassageBeliefStore.ingest`.

---

## What is in scope (this milestone)

- **Service:** `multi_robot_mission_stack_interfaces/srv/ProduceSemanticBlockedPassageV35` — `json_request` / `json_response` (UTF-8 JSON).
- **Core (no ROS):** `semantic_handoff_core_v35.py` — strict request envelope, explicit `assembly_timestamp_utc_iso` and `ingest_now_utc_iso`, optional `use_deterministic_fake_adapter` for offline proof.
- **Node:** `semantic-production-handoff-v35` — single role; owns one in-process store (allowlist via parameters, same pattern as transport receiver).

---

## What is not claimed

- Live **LLM → multi-robot relay** or fleet semantics.
- Changes to **default blocked_passage transport topic** or receiver node.
- Production readiness, prompt tuning, or second fact type.

---

## Artifacts

- `multi_robot_mission_stack/semantic/semantic_handoff_core_v35.py`
- `multi_robot_mission_stack/demo/semantic_production_handoff_node_v35.py`
- `multi_robot_mission_stack_interfaces/srv/ProduceSemanticBlockedPassageV35.srv`
- Tests: `tests/test_semantic_handoff_core_v35.py` (offline)

---

## Related

- [mission_control_semantic_checkpoint_through_v34.md](mission_control_semantic_checkpoint_through_v34.md)
- [mission_control_v3_3_llm_fact_production.md](mission_control_v3_3_llm_fact_production.md)
