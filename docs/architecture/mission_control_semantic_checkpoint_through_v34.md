# Mission Control — Semantic line checkpoint through V3.4 (closed / paused)

## Purpose

Single engineering checkpoint for the **bounded semantic + LLM-assist line** from frozen **`blocked_passage`** through **local gates**, **adapter**, **harness**, and **production-to-ingest proof**. The semantic line is **paused** here: no live LLM→ROS relay claim, no production readiness claim.

**Foundation (separate checkpoints):** V3.0.1 schema/store/policy substrate and V3.1 live relay demo — [mission_control_v3_0_1_checkpoint.md](mission_control_v3_0_1_checkpoint.md), [mission_control_v3_1_checkpoint.md](mission_control_v3_1_checkpoint.md).

**V3.3 contract:** [mission_control_v3_3_llm_fact_production.md](mission_control_v3_3_llm_fact_production.md). **V3.3 line detail:** [mission_control_v3_3_checkpoint.md](mission_control_v3_3_checkpoint.md).

---

## Closed slices (semantic package)

| Slice | What it proved (local unless noted) |
|-------|-------------------------------------|
| **V3.3** | Allowlisted `llm_context` + strict `llm_candidate` JSON; boundary; deterministic assembly; `validate_blocked_passage_record`; classified gate outcomes. |
| **V3.3.a** | Real-model adapter returns raw text into **unchanged** gates; `adapter_failure` path; provider code isolated. |
| **V3.3.b** | Repeated trials over fixed fixtures; outcome/reason counts only (`run_llm_eval_harness_v33b`, optional runner `--live`). |
| **V3.4** | After `accepted_record`, same dict admitted via **existing** `BlockedPassageBeliefStore.ingest` only; terminal ingest outcomes (`ingest_stored`, `ingest_duplicate_ignored`, `ingest_rejected`); **no** edits to boundary, assembler, adapter, or store ingest logic. |

**V3.5 (ROS handoff extension):** one JSON-in/JSON-out service façade over the **unchanged** V3.4 path — [mission_control_v3_5_checkpoint.md](mission_control_v3_5_checkpoint.md). Does not replace V3.0.1 transport topics.

**V3.6 (ROS runtime proof):** built workspace + manual `ros2 service call` evidence for the V3.5 seam — [mission_control_v3_6_checkpoint.md](mission_control_v3_6_checkpoint.md).

---

## Proven through V3.4 (engineering)

- One **bounded** route: candidate text → frozen V3.3.a pipeline → optional **unchanged** V3.0.1 ingest.
- Gate failures **never** call `ingest`; only **`accepted_record`** does.
- Store semantics exercised in tests: **duplicate-ignore**, **allowlist**, **TTL-at-ingest**, **active query** unchanged from V3.0.1 behavior.
- Classical vs semantic separation preserved: deterministic code owns validation, assembly, and admission **shape**; LLM is candidate-only.

---

## Intentionally not proven

- Live **LLM → ROS transport → robot** path for this route.
- End-to-end **policy execution in a running stack** using LLM-originated facts (beyond local store/query tests).
- Robustness across providers, prompts, or environments; acceptance rates as a product guarantee.
- Second fact type, schema broadening, or agentic orchestration.

---

## Out of scope while paused

- Reopening V3.3 parser, assembler, adapter, or harness for tuning.
- Transport redesign, bridge overload, policy seam changes for this track.
- Prompt optimization, provider bake-offs, LangGraph expansion for semantic production.

---

## Artifacts (code)

**V3.3 line** (`multi_robot_mission_stack/semantic/`):  
`llm_candidate_boundary_v33.py`, `llm_blocked_passage_assembler_v33.py`, `llm_candidate_eval_v33.py`, `llm_real_adapter_v33.py`, `llm_eval_harness_v33b.py`

**V3.4:** `semantic_production_ingest_v34.py` — `produce_and_ingest_blocked_passage_v34`

**Runner:** `scripts/run_llm_eval_harness_v33b.py`

**Tests** (`tests/`):  
`test_llm_candidate_boundary_v33.py`, `test_llm_blocked_passage_assembler_v33.py`, `test_llm_candidate_eval_v33.py`, `test_llm_real_adapter_v33.py`, `test_llm_eval_harness_v33b.py`, `test_semantic_production_ingest_v34.py`

**Imports:** `tests/conftest.py` prepends `src/multi_robot_mission_stack`.

---

## Regression (offline)

```bash
cd /path/to/HDNS && PYTHONPATH= python3 -m pytest \
  tests/test_llm_candidate_boundary_v33.py \
  tests/test_llm_blocked_passage_assembler_v33.py \
  tests/test_llm_candidate_eval_v33.py \
  tests/test_llm_real_adapter_v33.py \
  tests/test_llm_eval_harness_v33b.py \
  tests/test_semantic_production_ingest_v34.py -q
```

Optional live adapter/harness checks remain **opt-in** (env flags + credentials); they do not define CI for this line.
