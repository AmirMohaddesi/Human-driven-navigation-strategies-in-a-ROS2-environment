# Mission Control V7.3 — Bounded real-LLM launch smoke for advisory alternate fallback (checkpoint)

## Strategic question

With **`use_deterministic_fake_adapter: false`** on the V4.3 handoff+bridge launch graph, does a
single sequential step with **`alternate_location_name`** behave consistently under real-adapter
selection: **no** alternate when no belief is admitted, and **one** alternate attempt when a
compliant ingest blocks the primary (opt-in)?

## Purpose

**Fallback realism on the model side only**—same topology as V7.2 / V6.3; no new coordinator logic.

## Topology

Unchanged: `semantic_handoff_mission_bridge_v43.launch.py`.

**No production code changes** for V7.3.

## Paths

| Path | Launch env | Handoff | Sequence | Expectation |
|------|------------|---------|----------|-------------|
| **Default** | `OPENAI_*` stripped | `use_fake=False` → `adapter_failure` | One step, `base` + alt `test_goal` | No advisory primary; `alternate_attempted` false |
| **Opt-in** | Key inherited | ingest stored | same | Advisory primary; `alternate_attempted` true; `result` is alternate dict |

Opt-in success depends on network/provider/compliant output (not guaranteed in CI). Alternate
navigate may still fail headlessly; opt-in test still requires `alternate_result` dict and
`result is alternate_result` as in V7.2.

## Tests

**Default:**

```bash
cd /path/to/HDNS
source install/setup.bash
python3 -m pytest tests/test_semantic_handoff_bridge_real_llm_coordinator_v73_alternate_ros.py::test_v73_e2e_real_adapter_missing_key_no_alternate_attempt -q --tb=short
```

**Opt-in:**

```bash
export OPENAI_API_KEY=...
export RUN_V7_3_LLM_SMOKE=1
python3 -m pytest tests/test_semantic_handoff_bridge_real_llm_coordinator_v73_alternate_ros.py::test_v73_e2e_real_llm_handoff_alternate_fallback -q --tb=short
```

**Regression:**

```bash
python3 -m pytest tests/test_semantic_handoff_bridge_coordinator_v72_alternate_ros.py \
  tests/test_semantic_handoff_bridge_real_llm_coordinator_v63_hard_stop_ros.py::test_v63_e2e_real_adapter_missing_key_sequence_no_advisory_hard_stop \
  -q --tb=short
```

## Related

- [mission_control_v7_2_checkpoint.md](mission_control_v7_2_checkpoint.md) (fake-adapter launch alternate)
- [mission_control_v6_3_checkpoint.md](mission_control_v6_3_checkpoint.md) (real-LLM hard-stop smoke)
