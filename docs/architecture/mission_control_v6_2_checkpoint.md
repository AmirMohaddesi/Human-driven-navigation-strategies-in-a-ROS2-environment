# Mission Control V6.2 — Launch/runtime proof for advisory hard-stop sequencing (checkpoint)

## Strategic question

On the **live** handoff→mirror→bridge path, does a first leg blocked by advisory `blocked_passage`
trigger **V6.1** `hard_stop_on_advisory_blocked` so a 2-step coordinator sequence with
`continue_on_failure=True` **does not** schedule the second leg?

## Purpose

**Runtime proof** of one bounded execution behavior—not new control strategies.

## Topology

Same as V5.4: `semantic_handoff_mission_bridge_v43.launch.py` (handoff mirror +
`mission_bridge_node` on `/semantic/blocked_passage_v43_e2e`, Nav2 wait skipped).

## Proof

| Case | Handoff | Sequence | Expectation |
|------|---------|----------|-------------|
| Hard stop | Valid ingest for `base` | `robot1→base`, `robot2→test_goal`; `continue_on_failure=True`, `hard_stop_on_advisory_blocked=True` | `steps_run==1`, `stopped_due_to_advisory_blocked==True` |
| Control | Invalid JSON (no ingest) | Same options | `steps_run==2`, `stopped_due_to_advisory_blocked==False`; first leg not advisory-classified |

Second location `test_goal` matches packaged `named_locations.yaml`.

## Tests

**Launch / runtime:**

```bash
cd /path/to/HDNS
source install/setup.bash
python3 -m pytest tests/test_semantic_handoff_bridge_coordinator_v62_hard_stop_ros.py -q --tb=short
```

**Local regression (no launch):**

```bash
python3 -m pytest tests/test_coordinator.py::test_v61_hard_stop_advisory_blocked_stops_despite_continue_on_failure \
  tests/test_semantic_handoff_bridge_coordinator_v54_ros.py -q --tb=short
```

## Out of scope (V6.2)

Fallback goals, retries, parallel/in-process sequence changes, policy/contract changes, real-LLM,
broader launch graphs.

## Related

- [mission_control_v6_1_checkpoint.md](mission_control_v6_1_checkpoint.md) (hard-stop semantics)
- [mission_control_v5_4_checkpoint.md](mission_control_v5_4_checkpoint.md) (same launch, single-leg coordinator)
