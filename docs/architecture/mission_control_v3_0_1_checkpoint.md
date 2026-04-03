# Mission Control V3.0.1 — Milestone checkpoint (closed)

## Purpose

This note **closes** V3.0.1 as **complete enough** for handoff: it records **what was proven in repository artifacts**, **what was intentionally not proven**, and **where the next decision belongs** (a **new** milestone). It does not extend scope.

**Design contract (frozen record, policy intent):** [mission_control_v3_0_1_semantic_fact_exchange.md](mission_control_v3_0_1_semantic_fact_exchange.md).

---

## What V3.0.1 proved

- **`blocked_passage` only:** one frozen semantic record shape (schema validation, UUID v4 rules, provenance, `ttl_sec`, `verification_status == unverified`).
- **Executable belief layer:** `BlockedPassageBeliefStore` — validate on ingest, **duplicate-ignore by `belief_id`**, optional `source_robot_id` allowlist, reject already-expired-at-ingest records (TTL + skew at ingest `now_utc`).
- **Single policy seam:** before **`navigate_to_named_location`**, **`has_active_blocked_passage(location_name, now_utc=...)`**; if active → **no client dispatch**, frozen structured blocked outcome; **no** map / TF / costmap mutation, **no** bridge API change, **no** cancel of already-accepted goals.
- **Explicit time on the mission path:** `MissionAgentFacade.handle_command(..., now_utc=...)` → `graph.invoke(..., now_utc=...)` → tools; **no** hidden `datetime.now()` in that policy path.
- **Local producer stub:** `build_blocked_passage_record_stub(...)` builds valid records from explicit inputs (timezone-aware `timestamp_utc`).
- **Single-hop transport:** JSON object in **`std_msgs/String`** on topic default **`/semantic/blocked_passage_v301`**; receive path calls the **same** `store.ingest` as local ingest helpers.
- **Single-process composition:** `BlockedPassageSharedStoreRuntime` holds **one** `BlockedPassageBeliefStore` shared by **`ingest_transport_payload`** and **`MissionTools(..., blocked_passage_store=...)`** / facade; `BlockedPassageTransportReceiverNode(store=...)` can inject that store.

---

## Accepted artifacts and tests

**Code (reference layout under `multi_robot_mission_stack`):**

- `semantic/blocked_passage_v301.py` — schema, TTL/skew, store, blocked outcome helper.
- `semantic/blocked_passage_local_stub_v301.py` — local record builder.
- `agent/mission_tools.py` — policy hook on named-location navigation.
- `agent/mission_graph.py`, `agent/graph_state.py` — `now_utc` threading.
- `agent/mission_agent_facade.py` — `handle_command(..., now_utc=...)`.
- `agent/blocked_passage_shared_runtime_v301.py` — shared-store composition.
- `transport/blocked_passage_json_v301.py` — encode/decode/ingest payload.
- `transport/blocked_passage_receiver_node.py`, `transport/blocked_passage_sender_node.py` — ROS 2 pub/sub shell (receiver uses ROS clock at receive time for ingest `now_utc`).

**Tests (deterministic, no live ROS demo required for this checkpoint):**

- `tests/test_semantic_blocked_passage_v301.py`
- `tests/test_mission_tools_blocked_passage_policy_v301.py`
- `tests/test_mission_agent_facade.py` (includes facade + `now_utc` cases)
- `tests/test_blocked_passage_transport_v301.py`
- `tests/test_blocked_passage_shared_composition_v301.py`

**Suggested regression command:**

```bash
cd /path/to/HDNS && python3 -m pytest \
  tests/test_semantic_blocked_passage_v301.py \
  tests/test_mission_tools_blocked_passage_policy_v301.py \
  tests/test_mission_agent_facade.py \
  tests/test_blocked_passage_transport_v301.py \
  tests/test_blocked_passage_shared_composition_v301.py -q
```

---

## Clock and activeness semantics

- **TTL + skew:** `TTL_SKEW_ALLOWANCE_SEC = 0.5` s; activeness uses `timestamp_utc` + `ttl_sec` + skew vs an explicit timezone-aware **`now_utc`** (see semantic module docstrings).
- **Ingest vs policy (frozen for this slice):**
  - **Ingest** uses **`now_utc`** to **reject** records that are **already outside** the TTL window at ingest time and to enforce validation/allowlist/duplicate rules. It **does not** replace later policy-time evaluation.
  - **Policy** (**whether navigation is blocked at a decision**) is determined only by **`has_active_blocked_passage(..., now_utc=...)`** at that decision instant against **stored** records. **Active vs expired for blocking** is therefore decided at **policy evaluation time** with explicit **`now_utc`**, not by a separate durable “active bit” set only at ingest.

---

## What remains explicitly out of scope (not proven in V3.0.1)

- **Live ROS end-to-end demo** (multi-machine, real clocks, bridge under load); **field K-trial** success metric runs.
- **Default production wiring** of shared store + receiver + `mission-agent` / coordinator in every launch file; composed runtime is **proven in tests**, not mandated for all entrypoints.
- **Multi-hop relay, gossip, CRDTs, merge** beyond duplicate-ignore by `belief_id`.
- **Trust / signing / reputation** beyond optional `source_robot_id` allowlist at ingest.
- **New fact types**, **schema changes**, **LLM / NL on the wire**, **bridge/service contract changes**.
- **Geometry truth:** beliefs as costmap obstacles, map edits, TF authority.
- **Fallback sophistication** beyond the frozen no-fallback blocked outcome dict.
- **Broad LangGraph** or orchestration redesign.

---

## Why V3.0.1 stops here

Manager direction: the milestone is **complete enough** — bounded **semantic belief → single gate → explicit time → local transport → shared-store composition** is demonstrated with **deterministic tests**. Further work (operational rollout, demos, additional milestones) is **intentionally** a **new** planning item, not continued polishing of V3.0.1.

---

## Next milestone decision

Choose **separately** (examples only — not committed by this checkpoint):

- Harden **deployment wiring** (which processes construct `BlockedPassageSharedStoreRuntime`, how ROS clock vs mission `now_utc` align in production), **or**
- Add a **second fact type** / schema revision under a **V3.0.2+** label, **or**
- **Operational validation** (scripted live swarm trial, metrics in success metric), **or**
- **Policy extensions** (confidence thresholds, allowlisted alternate goals) only with an explicit new contract.

**Handoff:** Treat any of the above as a **new milestone** with its own scope guard; **do not** silently expand V3.0.1.

---

## Related documents

- [mission_control_v3_0_1_semantic_fact_exchange.md](mission_control_v3_0_1_semantic_fact_exchange.md) — frozen schema and normative intent (design note).
