# Mission Control V6.3 — Bounded real-LLM launch smoke for advisory hard-stop sequencing (checkpoint)

## Strategic question

With **`use_deterministic_fake_adapter: false`** on the V4.3 handoff+bridge launch graph, does a
**2-leg** coordinator sequence with **`continue_on_failure=True`** and
**`hard_stop_on_advisory_blocked=True`** behave consistently: **no** advisory hard stop when no
belief is admitted, and **hard stop** after the first leg when a compliant real ingest blocks
`base`?

## Purpose

**Execution realism on the model side only** — same topology as V6.2 / V5.5; no new control or
contracts.

## Topology

Unchanged: `semantic_handoff_mission_bridge_v43.launch.py`.

**No production code changes** for V6.3.

## Paths

| Path | Launch subprocess env | Handoff | Sequence | Expectation |
|------|------------------------|---------|----------|-------------|
| **Default** | `OPENAI_*` stripped | `use_fake=False` → `adapter_failure` | 2 legs, hard-stop flag on | Both legs run; `stopped_due_to_advisory_blocked=False` |
| **Opt-in** | Key inherited | ingest stored | same | One leg; `stopped_due_to_advisory_blocked=True` (if provider succeeds) |

## Tests

**Default (CI-friendly when ROS workspace available):**

```bash
cd /path/to/HDNS
source install/setup.bash
python3 -m pytest tests/test_semantic_handoff_bridge_real_llm_coordinator_v63_hard_stop_ros.py::test_v63_e2e_real_adapter_missing_key_sequence_no_advisory_hard_stop -q --tb=short
```

**Opt-in:**

```bash
export OPENAI_API_KEY=...
export RUN_V6_3_LLM_SMOKE=1
python3 -m pytest tests/test_semantic_handoff_bridge_real_llm_coordinator_v63_hard_stop_ros.py::test_v63_e2e_real_llm_handoff_sequence_hard_stops_after_advisory_leg -q --tb=short
```

**Regression:**

```bash
python3 -m pytest tests/test_semantic_handoff_bridge_real_llm_coordinator_v55_ros.py::test_v55_e2e_real_adapter_missing_key_handoff_coordinator_unclassified \
  tests/test_semantic_handoff_bridge_coordinator_v62_hard_stop_ros.py -q --tb=short
```

## Related

- [mission_control_v6_2_checkpoint.md](mission_control_v6_2_checkpoint.md) (fake-adapter launch hard stop)
- [mission_control_v5_5_checkpoint.md](mission_control_v5_5_checkpoint.md) (real-LLM single-leg reporting)
