# Mission Control V8.7 — bounded local transport payload and ingest helper for degraded_passage (checkpoint)

## Strategic question

Can `degraded_passage` support one bounded deterministic transport-ready JSON payload and one
local ingest helper using the V8.3 store, without ROS runtime, bridge/coordinator, or execution
meaning?

## Scope

V8.7 adds pure-Python encode/decode + ingest helper only (mirrors `blocked_passage_json_v301`
discipline). No topic/service registration, no launch, no handoff/bridge/coordinator wiring.

## Deliverables

- `src/multi_robot_mission_stack/multi_robot_mission_stack/transport/degraded_passage_json_v87.py`
- `src/multi_robot_mission_stack/multi_robot_mission_stack/transport/__init__.py` (exports)
- `tests/test_degraded_passage_transport_v87.py`

## Test command

```bash
cd /path/to/HDNS
python3 -m pytest tests/test_degraded_passage_transport_v87.py -q --tb=short
```

## Explicitly unchanged

- no live ROS integration
- no mission policy or navigation gates for `degraded_passage`
- no generalized multi-fact wire framework

## Related

- `blocked_passage_json_v301.py` (pattern reference)
- [mission_control_v8_6_checkpoint.md](mission_control_v8_6_checkpoint.md)
