# Mission Control V9.7 — bounded live sequence summary propagation for degraded advisory (checkpoint)

## Strategic question

Can a live sequence return expose **one bounded, additive** degraded observation at the
**sequence-output** level, without changing dispatch, blocked/failure semantics, or turning
degraded into control, classification, hard-stop, or fallback?

## V9.7 answer (this milestone)

Yes, with a **minimal** rule (not multi-step aggregation):

- Step-level payloads remain on ``steps[i]["result"]`` (V9.6).
- **Additionally**, when any executed step's ``result`` contains ``degraded_passage_advisory``,
  the sequence dict may include the **same key** at the **top level** as a **shallow copy** of the
  **first** such step in **execution order**. If no step carries the field, the top-level key is
  **absent**.
- No merge of entries across locations, no TTL fusion, no new classification — observation only.

Rationale vs. alternatives: a boolean would drop payload detail; copying only the **last** step
would drop earlier advisories without a stated policy; full merge is out of scope. **First-step
with advisory** is the smallest deterministic summary that matches "bounded" and keeps one
additive field aligned with the existing ``degraded_passage_advisory`` shape.

## Scope

- ``assign_named_sequence`` return dict only (+ small helper in ``coordinator.py``).
- Unit + live test assertions; this checkpoint doc.

## Tests

```bash
cd /path/to/HDNS
. install/setup.bash
python3 -m pytest \
  tests/test_coordinator.py::test_v92_assign_named_sequence_preserves_degraded_on_step_result \
  tests/test_coordinator.py::test_v97_assign_named_sequence_no_top_level_degraded_when_steps_lack_advisory \
  tests/test_coordinator_bridge_degraded_advisory_v96_ros.py \
  -q --tb=short
```

## Related

- [mission_control_v9_6_checkpoint.md](mission_control_v9_6_checkpoint.md)
