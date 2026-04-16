# Mission Control V8.8 — bounded local candidate→transport→admission→advisory composition (checkpoint)

## Strategic question

Can `degraded_passage` complete one full local transport-aware path — candidate, assembly, JSON
encode, transport ingest, advisory report — without ROS runtime, bridge/coordinator, or execution
meaning?

## Scope

V8.8 is local composition proof only (focused tests). No new production pipeline or ROS wiring.

## Deliverables

- `tests/test_semantic_degraded_passage_compose_transport_v88.py`
- this checkpoint document

## Test command (smallest bounded V8.8 set)

```bash
cd /path/to/HDNS
python3 -m pytest tests/test_semantic_degraded_passage_compose_transport_v88.py -q --tb=short
```

## Related

- [mission_control_v8_7_checkpoint.md](mission_control_v8_7_checkpoint.md)
- [mission_control_v8_6_checkpoint.md](mission_control_v8_6_checkpoint.md)
