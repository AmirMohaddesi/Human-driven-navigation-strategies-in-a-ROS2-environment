# Mission Control V9.2 — bounded coordinator propagation of degraded advisory observation (checkpoint)

## Strategic question

Can the same additive `degraded_passage_advisory` payload from the initial navigate dict be
preserved on coordinator-facing leg results, without changing dispatch, blocked/failure semantics,
or turning degraded into a classification or control signal?

## V9.2 answer (this milestone)

Yes, at **`assign_named_navigation`** only: the initial `facade.handle_command(navigate)` dict
may contain `degraded_passage_advisory` (V9.1 / MissionTools). Coordinator leg-shaped returns
(success, navigate-fail, and wait-exception paths) merge that key forward from **`nav`** using
`_leg_dict_with_degraded_advisory_from_navigate`.

Because **`assign_named_sequence`** / **`assign_named_parallel`** embed `assign_named_navigation`
outputs in `step["result"]`, those summaries gain the same additive field **without** separate
coordinator decision logic.

## Scope

- `coordinator.py` merge helper + `assign_named_navigation` return paths only.
- No bridge, MissionTools redesign, `navigate_failure_kind` changes, or degraded blocking.

## Tests

```bash
cd /path/to/HDNS
python3 -m pytest \
  tests/test_coordinator.py::test_v92_assign_named_navigation_propagates_degraded_advisory_on_success \
  tests/test_coordinator.py::test_v92_assign_named_navigation_propagates_degraded_on_navigate_fail_path \
  tests/test_coordinator.py::test_v92_assign_named_sequence_preserves_degraded_on_step_result \
  -q --tb=short
```

## Related

- [mission_control_v9_1_checkpoint.md](mission_control_v9_1_checkpoint.md)
