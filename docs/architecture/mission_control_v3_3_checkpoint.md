# Mission Control V3.3 line — Milestone checkpoint (paused)

## Purpose

Checkpoint the **paused** bounded **V3.3** line: **local deterministic gates**, **V3.3.a** real-model adapter seam, and **V3.3.b** repeated-trial harness. This is **not** a claim of live semantic production, ROS integration, or operational readiness.

Normative contract: [mission_control_v3_3_llm_fact_production.md](mission_control_v3_3_llm_fact_production.md).

---

## Closed slices (local / bounded)

**V3.3 — local gate stack**

- Allowlisted `llm_context`, strict single-object JSON `llm_candidate`, explicit reject tokens.
- Deterministic assembly into V3.0.1 `blocked_passage` with `validate_blocked_passage_record` and `final_schema_reject`.
- Local eval: `evaluate_llm_candidate_local_v33`, classified outcomes (`accepted_record`, `boundary_reject`, `final_schema_reject`).

**V3.3.a — minimal real-model adapter**

- Raw text in / out; provider code isolated in `llm_real_adapter_v33.py`.
- `evaluate_llm_candidate_via_adapter_v33` runs the **unchanged** gate stack; `adapter_failure` and adapter reason tokens for transport/empty/provider errors.
- Optional live OpenAI-compatible path; default unit tests are **offline**.

**V3.3.b — bounded evaluation harness**

- `run_llm_eval_harness_v33b`: fixed fixtures × repeated trials, outcome and reason counts only.
- Runner: `scripts/run_llm_eval_harness_v33b.py` (`--trials`, optional `--live` + `OPENAI_API_KEY`).

---

## What is explicitly not claimed here

- Repeated-run **acceptance rates** as a product guarantee.
- **ROS**, **store**, **transport**, or **policy** execution on LLM-produced candidates.
- **Production** robustness, prompt tuning, or broad agent behavior.

---

## Artifacts

**Semantic modules** (`multi_robot_mission_stack/semantic/`)

- `llm_candidate_boundary_v33.py`
- `llm_blocked_passage_assembler_v33.py`
- `llm_candidate_eval_v33.py`
- `llm_real_adapter_v33.py`
- `llm_eval_harness_v33b.py`

**Tests** (repo `tests/`)

- `test_llm_candidate_boundary_v33.py`
- `test_llm_blocked_passage_assembler_v33.py`
- `test_llm_candidate_eval_v33.py`
- `test_llm_real_adapter_v33.py` (includes opt-in live test gated by `HDNS_V33A_LIVE_ADAPTER=1`)
- `test_llm_eval_harness_v33b.py`

**Runner**

- `scripts/run_llm_eval_harness_v33b.py`

**Test path**

- `tests/conftest.py` prepends `src/multi_robot_mission_stack` so `multi_robot_mission_stack` imports without colcon.

---

## Deterministic regression (offline)

```bash
cd /path/to/HDNS && PYTHONPATH= python3 -m pytest \
  tests/test_llm_candidate_boundary_v33.py \
  tests/test_llm_blocked_passage_assembler_v33.py \
  tests/test_llm_candidate_eval_v33.py \
  tests/test_llm_real_adapter_v33.py \
  tests/test_llm_eval_harness_v33b.py -q
```

Optional live provider checks remain **opt-in** and do not block CI.

---

## Related documents

- [mission_control_semantic_checkpoint_through_v34.md](mission_control_semantic_checkpoint_through_v34.md) — semantic + LLM-assist line closed through V3.4 (production-to-ingest).
- [mission_control_v3_0_1_checkpoint.md](mission_control_v3_0_1_checkpoint.md)
- [mission_control_v3_1_checkpoint.md](mission_control_v3_1_checkpoint.md)
