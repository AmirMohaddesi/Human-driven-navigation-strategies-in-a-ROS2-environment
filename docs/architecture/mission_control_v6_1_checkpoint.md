# Mission Control V6.1 — Opt-in hard stop on advisory blocked_passage (checkpoint)

## Strategic question

If a sequential team mission runs with `continue_on_failure=True`, can an explicit opt-in stop
scheduling further legs after a failure whose leg result includes
`navigate_failure_kind == advisory_blocked_passage`, without changing belief admission, transport,
bridge refusal, or static allowlist policy?

## Purpose (V6 family)

First **execution** milestone: change **what runs next** at the orchestration layer, not new
navigation targets or replanning.

## Behavior

| Parameter | Default | Effect |
|-----------|---------|--------|
| `hard_stop_on_advisory_blocked` | `False` | Same as pre-V6.1 `assign_named_sequence` |
| `hard_stop_on_advisory_blocked` | `True` | After a leg with advisory `navigate_failure_kind`, break the loop even if `continue_on_failure` is `True` |

Additive summary fields on sequence results:

- `hard_stop_on_advisory_blocked` — echo of the flag
- `stopped_due_to_advisory_blocked` — `True` only when the loop ended because of the advisory hard stop

`stopped_early` is `True` when fewer than `total_steps` legs ran and there was at least one failure,
including the case `continue_on_failure=True` with advisory hard stop.

## Spec / options

Mission spec v1 `options` may include `hard_stop_on_advisory_blocked` (boolean). Threaded through
`run_team_named_mission_spec` → `assign_team_named_mission` → `assign_named_sequence`.

## Tests (local)

```bash
cd /path/to/HDNS
python3 -m pytest \
  tests/test_coordinator.py::test_v61_hard_stop_advisory_blocked_stops_despite_continue_on_failure \
  tests/test_coordinator.py::test_v61_continue_on_failure_still_runs_after_generic_failure_with_hard_stop_flag \
  tests/test_coordinator.py::test_v61_run_team_named_mission_spec_threads_hard_stop_option \
  tests/test_coordinator.py::test_assign_named_sequence_two_legs \
  -q --tb=short
```

Broader regression: `python3 -m pytest tests/test_coordinator.py -q --tb=short`

## Out of scope (V6.1)

Parallel batches, `sequence_utils` parity, retries, fallback goals, policy allowlist changes,
bridge `.srv`, launch proofs.

## Related

- V5.1 classification: `navigate_failure_kind` on coordinator legs
- V6 proposal: bounded execution behavior family
