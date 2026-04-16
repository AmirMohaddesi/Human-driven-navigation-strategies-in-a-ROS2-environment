# Mission Control V11.3 — local belief store for `shared_space_caution` (checkpoint)

## Strategic question

Can the frozen `shared_space_caution` fact type reuse the same bounded local admission discipline as
the prior fact types—including deterministic ingest, active-window semantics, duplicate-ignore
behavior, and source allowlist checks—without introducing runtime transport, bridge/coordinator use,
or execution meaning?

## Scope

V11.3 extends the existing `shared_space_caution_v111` module with an in-memory store only:

- `SharedSpaceCautionBeliefStore` with `ingest` and `has_active_shared_space_caution`
- `is_shared_space_caution_active` helper (explicit ``now_utc`` for TTL + skew)
- `SharedSpaceCautionIngestResult` and `ActiveSharedSpaceCautionQueryResult` dataclasses

No ROS, transport, bridge, coordinator, or mission-layer changes.

## Deliverables

- `src/multi_robot_mission_stack/multi_robot_mission_stack/semantic/shared_space_caution_v111.py`
  (store + active helper)
- `tests/test_semantic_shared_space_caution_belief_store_v113.py`
- `multi_robot_mission_stack.semantic` exports for the new public types

## Test command

```bash
cd /path/to/HDNS
python3 -m pytest \
  tests/test_semantic_shared_space_caution_belief_store_v113.py \
  tests/test_semantic_shared_space_caution_v111.py \
  tests/test_semantic_contract_v11_1_shared_space_caution_docsync.py \
  -q --tb=short
```

## Explicitly unchanged

- no transport or JSON envelope for `shared_space_caution`
- no bridge/coordinator/MissionTools wiring
- no execution, logging, or navigation policy hooks
- no new fact types or taxonomy changes

## Related

- [mission_control_v11_4_checkpoint.md](mission_control_v11_4_checkpoint.md) (advisory reporting)
- [mission_control_v11_2_checkpoint.md](mission_control_v11_2_checkpoint.md)
- [mission_control_v11_1_shared_space_caution_contract.md](mission_control_v11_1_shared_space_caution_contract.md)
