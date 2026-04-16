# Mission Control V8.5 — bounded local candidate-to-record assembly for degraded_passage (checkpoint)

## Strategic question

Can `degraded_passage` support one bounded deterministic local candidate-to-record assembly path,
with validator-backed final acceptance, without runtime integration or execution semantics?

## Scope

V8.5 is local production discipline only:

- `DegradedPassageAssemblyCandidate` (narrow producer surface)
- `assemble_degraded_passage_record_from_candidate(...)`
- `DegradedPassageAssembledAccept` / `DegradedPassageAssembledReject`
- candidate pre-checks + mandatory `validate_degraded_passage_record` on the assembled dict

No transport, handoff, bridge, coordinator, launch, or mission policy wiring.

## Deliverables

- `src/multi_robot_mission_stack/multi_robot_mission_stack/semantic/degraded_passage_candidate_v85.py`
- `src/multi_robot_mission_stack/multi_robot_mission_stack/semantic/__init__.py` (exports)
- `tests/test_semantic_degraded_passage_candidate_v85.py`

## Test command

```bash
cd /path/to/HDNS
python3 -m pytest \
  tests/test_semantic_degraded_passage_candidate_v85.py \
  tests/test_semantic_degraded_passage_v81.py \
  tests/test_semantic_contract_v8_1_degraded_passage_docsync.py \
  -q --tb=short
```

## Explicitly unchanged

- `degraded_passage` remains advisory-only; assembly does not connect to navigation or stores.
- No generalized candidate framework beyond this fact-specific module.

## Related

- [mission_control_v8_4_checkpoint.md](mission_control_v8_4_checkpoint.md)
- [mission_control_v8_1_degraded_passage_contract.md](mission_control_v8_1_degraded_passage_contract.md)
