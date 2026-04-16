# Mission Control V8.6 — bounded local candidate→admission→advisory composition for degraded_passage (checkpoint)

## Strategic question

Can `degraded_passage` complete one full local proof path — candidate, deterministic assembly,
validator-backed ingest, and advisory report — without runtime transport, bridge/coordinator, or
execution meaning?

## Scope

V8.6 is local composition proof only, implemented as focused tests (no new production pipeline layer).

## Deliverables

- `tests/test_semantic_degraded_passage_compose_v86.py`
- this checkpoint document

## Test command (smallest bounded V8.6 set)

```bash
cd /path/to/HDNS
python3 -m pytest tests/test_semantic_degraded_passage_compose_v86.py -q --tb=short
```

Optional quick regression on prior V8 slices:

```bash
python3 -m pytest \
  tests/test_semantic_degraded_passage_candidate_v85.py \
  tests/test_semantic_degraded_passage_advisory_v84.py \
  tests/test_semantic_degraded_passage_store_v83.py \
  -q --tb=short
```

## Explicitly unchanged

- no transport, handoff, bridge, coordinator, or launch wiring
- no mission policy or execution semantics
- no generalized composition framework

## Related

- [mission_control_v8_5_checkpoint.md](mission_control_v8_5_checkpoint.md)
- [mission_control_v8_3_checkpoint.md](mission_control_v8_3_checkpoint.md)
