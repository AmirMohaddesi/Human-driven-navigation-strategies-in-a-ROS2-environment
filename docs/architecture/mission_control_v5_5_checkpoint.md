# Mission Control V5.5 — Bounded real-LLM end-to-end reporting smoke (checkpoint)

## Strategic question

With **`use_deterministic_fake_adapter: false`** on the same V4.3 launch graph (handoff mirror +
bridge advisory), does the **coordinator** path still behave consistently: **unclassified** when
no belief is admitted, and **`navigate_failure_kind == advisory_blocked_passage`** when a compliant
real-model ingest occurs?

## Purpose

**Reporting realism on the model side only** — exercise the real adapter branch without new control,
contracts, or evaluation scope.

## Topology

**Unchanged:** `semantic_handoff_mission_bridge_v43.launch.py` (same as V4.3 / V4.4 / V5.4).

No launch or node code changes are required for V5.5.

## Automated paths

| Path | Launch env | Handoff request | Coordinator | Expectation |
|------|------------|-----------------|-------------|-------------|
| **Default** | `OPENAI_*` stripped from subprocess | `use_deterministic_fake_adapter: false` | `assign_named_navigation(robot1, base)` | Handoff `adapter_failure`; no `navigate_failure_kind` |
| **Opt-in** | Parent env inherited (needs key) | same | same | `ingest_stored` → blocked message + `navigate_failure_kind` if provider succeeds |

## Tests

**Default (CI-friendly when ROS workspace is available):**

```bash
cd /path/to/HDNS
source install/setup.bash   # if not already sourced
python3 -m pytest tests/test_semantic_handoff_bridge_real_llm_coordinator_v55_ros.py::test_v55_e2e_real_adapter_missing_key_handoff_coordinator_unclassified -q --tb=short
```

**Opt-in real success (network + provider + compliant output not guaranteed):**

```bash
export OPENAI_API_KEY=...   # or your provider key as required by llm_real_adapter_v33
export RUN_V5_5_LLM_SMOKE=1
python3 -m pytest tests/test_semantic_handoff_bridge_real_llm_coordinator_v55_ros.py::test_v55_e2e_real_llm_handoff_coordinator_navigate_failure_kind -q --tb=short
```

**Regression (fake-adapter E2E + prior LLM smoke):**

```bash
python3 -m pytest tests/test_semantic_handoff_bridge_coordinator_v54_ros.py \
  tests/test_semantic_handoff_bridge_real_llm_v44_ros.py::test_v44_e2e_real_adapter_missing_key_handoff_bridge_not_blocked -q --tb=short
```

## Related

- [mission_control_v5_4_checkpoint.md](mission_control_v5_4_checkpoint.md) (fake-adapter handoff→coordinator)
- V4.4 bridge-only real-LLM smoke: `tests/test_semantic_handoff_bridge_real_llm_v44_ros.py`
