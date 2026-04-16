# Mission Control V11.4 — local advisory query/reporting for `shared_space_caution` (checkpoint)

## Strategic question

Can the frozen and locally admitted `shared_space_caution` fact type be surfaced through one
stable, machine-readable local advisory query/reporting shape, without introducing any execution
consequence, bridge/coordinator wiring, or runtime transport integration?

## Scope

V11.4 adds advisory-only reporting on top of the existing `SharedSpaceCautionBeliefStore`:

- `SharedSpaceCautionAdvisoryEntry` and `SharedSpaceCautionAdvisoryReport` dataclasses
- `query_advisory_shared_space_caution` on the store
- frozen report envelope literal `v11.4.advisory.1`

No ROS, transport, bridge, coordinator, or mission-layer changes.

## Deliverables

- `src/multi_robot_mission_stack/multi_robot_mission_stack/semantic/shared_space_caution_v111.py`
  (advisory types + query method)
- `tests/test_semantic_shared_space_caution_advisory_v114.py`
- `multi_robot_mission_stack.semantic` exports for advisory types and report schema constant

## Test command

```bash
cd /path/to/HDNS
python3 -m pytest \
  tests/test_semantic_shared_space_caution_advisory_v114.py \
  tests/test_semantic_shared_space_caution_belief_store_v113.py \
  tests/test_semantic_shared_space_caution_v111.py \
  tests/test_semantic_contract_v11_1_shared_space_caution_docsync.py \
  -q --tb=short
```

## Explicitly unchanged

- no transport or JSON wire format for `shared_space_caution`
- no bridge/coordinator/MissionTools integration
- no navigation gates, blocking, hard-stop, or fallback behavior

## Related

- [mission_control_v11_6_local_line_closure_checkpoint.md](mission_control_v11_6_local_line_closure_checkpoint.md) — full local chain proof (V11.6)
- [mission_control_v11_5_checkpoint.md](mission_control_v11_5_checkpoint.md) (candidate assembly)
- [mission_control_v11_3_checkpoint.md](mission_control_v11_3_checkpoint.md)
- [mission_control_v11_1_shared_space_caution_contract.md](mission_control_v11_1_shared_space_caution_contract.md)
