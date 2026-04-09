# Mission Control V8.2 — deterministic local validator scaffold for degraded_passage (checkpoint)

## Strategic question

Can the frozen V8.1 `degraded_passage` contract be enforced by deterministic local validation
only, with accept/reject behavior aligned to the V8.1 examples, and without runtime integration?

## Scope

V8.2 adds local validation and tests only:

- `validate_degraded_passage_record` function
- fixture-aligned accept/reject tests

No launch/transport/bridge/coordinator/execution changes are included.

## Deliverables

- `src/multi_robot_mission_stack/multi_robot_mission_stack/semantic/degraded_passage_v81.py`
- `tests/test_semantic_degraded_passage_v81.py`

## Test command

```bash
cd /path/to/HDNS
python3 -m pytest \
  tests/test_semantic_degraded_passage_v81.py \
  tests/test_semantic_contract_v8_1_degraded_passage_docsync.py \
  -q --tb=short
```

## Explicitly unchanged

- no ROS nodes or launch graph changes
- no handoff/bridge integration
- no coordinator/mission behavior changes
- no policy authority expansion for `degraded_passage`
- no new fact types beyond `degraded_passage`

## Related

- [mission_control_v8_1_checkpoint.md](mission_control_v8_1_checkpoint.md)
- [mission_control_v8_1_degraded_passage_contract.md](mission_control_v8_1_degraded_passage_contract.md)
