# Mission Control V8.3 — deterministic local store/admission scaffold for degraded_passage (checkpoint)

## Strategic question

Can `degraded_passage` reuse bounded local admission/store discipline (validator-backed ingest,
duplicate-ignore, active-window semantics, source allowlist) with no runtime integration?

## Scope

V8.3 is local semantic scaffolding only:

- fact-specific in-memory store for `degraded_passage`
- deterministic ingest result shape
- active query by named location
- focused unit tests

No ROS transport/launch/bridge/coordinator/mission behavior changes are included.

## Deliverables

- `src/multi_robot_mission_stack/multi_robot_mission_stack/semantic/degraded_passage_v81.py`
  - `IngestResult`
  - `ActiveDegradedQueryResult`
  - `is_degraded_passage_active(...)`
  - `DegradedPassageBeliefStore`
- `tests/test_semantic_degraded_passage_store_v83.py`

## Test command

```bash
cd /path/to/HDNS
python3 -m pytest \
  tests/test_semantic_degraded_passage_v81.py \
  tests/test_semantic_degraded_passage_store_v83.py \
  tests/test_semantic_contract_v8_1_degraded_passage_docsync.py \
  -q --tb=short
```

## Explicitly unchanged

- no runtime hooks
- no policy/execution meaning for `degraded_passage`
- no multi-fact generalized framework
- no new fact types

## Related

- [mission_control_v8_1_degraded_passage_contract.md](mission_control_v8_1_degraded_passage_contract.md)
- [mission_control_v8_2_checkpoint.md](mission_control_v8_2_checkpoint.md)
