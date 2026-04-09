# Mission Control V7.2 — Launch/runtime proof for advisory alternate fallback (checkpoint)

## Strategic question

On the **live** handoff→mirror→bridge stack, does an advisory-blocked **primary** named navigate
cause the coordinator to run **exactly one** predeclared **alternate** navigate (V7.1), with
correct step record fields?

## Purpose

**Runtime proof** of V7.1 alternate behavior end-to-end—not new fallback logic or trees.

## Topology

Same as V5.4 / V6.2: `semantic_handoff_mission_bridge_v43.launch.py`.

**No production code changes** for V7.2.

## Proof

| Case | Handoff | Sequence | Expectation |
|------|---------|----------|-------------|
| Alternate path | Valid ingest for `base` | One step: `robot1→base`, `alternate_location_name=test_goal` | Primary advisory; `alternate_attempted`; `alternate_result` dict; `result` is alternate dict |
| Control | Invalid JSON (no ingest) | Same step | Primary not advisory; `alternate_attempted` false; `result` is primary |

**Headless note:** The alternate leg may still **fail** (e.g. Nav2 action server unavailable). V7.2
asserts the **coordinator invoked** the alternate navigate and recorded the alternate outcome as
effective, not that navigation completes successfully.

## Tests

**Launch / runtime:**

```bash
cd /path/to/HDNS
source install/setup.bash
python3 -m pytest tests/test_semantic_handoff_bridge_coordinator_v72_alternate_ros.py -q --tb=short
```

**Local regression:**

```bash
python3 -m pytest tests/test_coordinator.py::test_v71_advisory_primary_triggers_single_alternate_navigate \
  tests/test_semantic_handoff_bridge_coordinator_v62_hard_stop_ros.py -q --tb=short
```

## Out of scope (V7.2)

Multiple alternates, trees, retries, real-LLM success guarantees, parallel/`sequence_utils`,
contract/store changes.

## Related

- [mission_control_v7_1_checkpoint.md](mission_control_v7_1_checkpoint.md) (alternate semantics)
- [mission_control_v6_2_checkpoint.md](mission_control_v6_2_checkpoint.md) (same launch graph)
