# Mission Control V11.8 — canonical local example bundle for `shared_space_caution`

**Static inspectability only.** These JSON files are **not** wire formats, DDS payloads, or
runtime truth. They mirror shapes produced by **existing local-only** public APIs for reviewer
orientation.

## Bundle location

`docs/architecture/examples/v11_8_shared_space_caution_local/`

| File | Maps to | Role |
|------|---------|------|
| `candidate.canonical.json` | **V11.5** `SharedSpaceCautionAssemblyCandidate` fields | Producer-side inputs **before** code-owned ids/timestamps. |
| `assembled_record.canonical.json` | **V11.5** output + **V11.2** contract | Full **V11.1** record dict from `assemble_shared_space_caution_record_from_candidate` (fixed clock + UUIDs). |
| `advisory_report.canonical.json` | **V11.4** report envelope | `SharedSpaceCautionAdvisoryReport.to_dict()` after **V11.3** `ingest` of the assembled record and `query_advisory_shared_space_caution` at the same `now_utc` (same chain as **V11.6**). |

## Deterministic parameters (frozen for the bundle)

- `timestamp_utc` (assembly): `2026-04-08T12:30:30Z`
- `belief_id`: `123e4567-e89b-42d3-a456-426614174000`
- `provenance.observation_id`: `223e4567-e89b-42d3-a456-426614174111`
- Store: `SharedSpaceCautionBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))`, ingest and query at the same UTC instant.

## Verification

Checked-in examples are guarded by:

```bash
cd /path/to/HDNS
python3 -m pytest tests/test_semantic_shared_space_caution_v11_8_examples_bundle_sync.py -q --tb=short
```

Full local suite (unchanged from V11.7):

```bash
python3 -m pytest \
  tests/test_semantic_shared_space_caution_local_chain_v116.py \
  tests/test_semantic_shared_space_caution_candidate_v115.py \
  tests/test_semantic_shared_space_caution_advisory_v114.py \
  tests/test_semantic_shared_space_caution_belief_store_v113.py \
  tests/test_semantic_shared_space_caution_v111.py \
  tests/test_semantic_contract_v11_1_shared_space_caution_docsync.py \
  tests/test_semantic_shared_space_caution_v11_8_examples_bundle_sync.py \
  -q --tb=short
```

## Out of scope (unchanged)

- Runtime, ROS, transport, bridge, coordinator, MissionTools, mission semantics.
- Any change to contract, TTL, duplicate, or store meaning.

## Related

- [mission_control_v11_7_local_proof_bundle_closure.md](mission_control_v11_7_local_proof_bundle_closure.md)
- [mission_control_v11_6_local_line_closure_checkpoint.md](mission_control_v11_6_local_line_closure_checkpoint.md)
- [mission_control_v11_5_checkpoint.md](mission_control_v11_5_checkpoint.md)
- [mission_control_v11_4_checkpoint.md](mission_control_v11_4_checkpoint.md)
- [mission_control_v11_1_shared_space_caution_contract.md](mission_control_v11_1_shared_space_caution_contract.md)
