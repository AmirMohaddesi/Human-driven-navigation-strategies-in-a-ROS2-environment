# Mission Control V11.5 — candidate-to-record assembly for `shared_space_caution` (checkpoint)

## Strategic question

Can the frozen `shared_space_caution` fact type support one bounded deterministic local
candidate-to-record assembly path, with validator-backed final record construction, without
introducing runtime transport, bridge/coordinator use, or execution semantics?

## Scope

V11.5 adds a fact-specific assembly module only:

- `SharedSpaceCautionAssemblyCandidate` (narrow producer surface)
- `assemble_shared_space_caution_record_from_candidate`
- accept/reject results with `CANDIDATE_INVALID` vs `final_schema_reject`

Code owns `belief_id`, `timestamp_utc`, `fact_type`, `schema_version`, `verification_status`, and
`provenance.observation_id` (mirroring the V8.5 degraded pattern). Final acceptance is always
`validate_shared_space_caution_record`.

No ROS, transport, bridge, coordinator, or mission-layer changes.

## Deliverables

- `src/multi_robot_mission_stack/multi_robot_mission_stack/semantic/shared_space_caution_candidate_v115.py`
- `tests/test_semantic_shared_space_caution_candidate_v115.py`
- `multi_robot_mission_stack.semantic` exports (aliased where names collide with degraded assembly)

## Test command

```bash
cd /path/to/HDNS
python3 -m pytest \
  tests/test_semantic_shared_space_caution_candidate_v115.py \
  tests/test_semantic_shared_space_caution_v111.py \
  tests/test_semantic_contract_v11_1_shared_space_caution_docsync.py \
  -q --tb=short
```

## Explicitly unchanged

- no transport or wire encoding for `shared_space_caution`
- no bridge/coordinator/MissionTools wiring
- no execution, navigation gates, or policy hooks

## Related

- [mission_control_v11_7_local_proof_bundle_closure.md](mission_control_v11_7_local_proof_bundle_closure.md) — **V11 local-only proof bundle / reviewer entry**
- [mission_control_v11_6_local_line_closure_checkpoint.md](mission_control_v11_6_local_line_closure_checkpoint.md) — assemble → ingest → active → advisory chain proof
- [mission_control_v11_4_checkpoint.md](mission_control_v11_4_checkpoint.md)
- [mission_control_v11_1_shared_space_caution_contract.md](mission_control_v11_1_shared_space_caution_contract.md)
