# Mission Control V8.4 — bounded local advisory query/reporting for degraded_passage (checkpoint)

## Strategic question

Can locally admitted `degraded_passage` beliefs be surfaced through one stable, machine-readable
local advisory query/reporting shape, with no execution consequence and no runtime integration?

## Scope

V8.4 is local observation only:

- `DegradedPassageAdvisoryEntry` / `DegradedPassageAdvisoryReport`
- `DegradedPassageBeliefStore.query_advisory_degraded_passage(...)`
- `DegradedPassageAdvisoryReport.to_dict()` for a fixed JSON-friendly envelope

No transport, handoff, bridge, coordinator, launch, or navigation policy changes.

## Deliverables

- `src/multi_robot_mission_stack/multi_robot_mission_stack/semantic/degraded_passage_v81.py`
- `src/multi_robot_mission_stack/multi_robot_mission_stack/semantic/__init__.py` (exports)
- `tests/test_semantic_degraded_passage_advisory_v84.py`

## Test command

```bash
cd /path/to/HDNS
python3 -m pytest \
  tests/test_semantic_degraded_passage_advisory_v84.py \
  tests/test_semantic_degraded_passage_v81.py \
  tests/test_semantic_degraded_passage_store_v83.py \
  tests/test_semantic_contract_v8_1_degraded_passage_docsync.py \
  -q --tb=short
```

## Explicitly unchanged

- `degraded_passage` remains advisory-only; report must not gate navigation or trigger alternates.
- No generalized multi-fact query framework.

## Related

- [mission_control_v8_3_checkpoint.md](mission_control_v8_3_checkpoint.md)
- [mission_control_v8_1_degraded_passage_contract.md](mission_control_v8_1_degraded_passage_contract.md)
