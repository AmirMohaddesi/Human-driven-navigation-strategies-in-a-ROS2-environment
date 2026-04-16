# Mission Control V9.6 — bounded live sequence-step propagation for degraded advisory (checkpoint)

## Strategic question

Can bridge-originated ``degraded_passage_advisory`` survive into **coordinator sequence step**
``result`` dicts on a **live** path, without changing dispatch, blocked/failure semantics, or
turning degraded into control or classification?

## V9.6 answer (this milestone)

Yes. ``assign_named_sequence`` already embeds each leg as ``steps[i]["result"]`` (the dict returned
by ``assign_named_navigation``). No new merge logic was required—only forwarding the V9.5
``client_rcl_node`` optional through ``assign_named_sequence`` into each ``assign_named_navigation``
call (primary and alternate) so the same bounded in-process bridge + executor topology works for
sequences.

## Scope

- ``assign_named_sequence`` keyword-only parameter + two call-site forwards.
- One-step live ROS test (positive + control); checkpoint doc.
- Parallel batch / multi-step scenarios intentionally out of scope.

## Tests

```bash
cd /path/to/HDNS
. install/setup.bash
python3 -m pytest tests/test_coordinator_bridge_degraded_advisory_v96_ros.py -q --tb=short
```

## Related

- [mission_control_v9_5_checkpoint.md](mission_control_v9_5_checkpoint.md)
- [mission_control_v9_4_checkpoint.md](mission_control_v9_4_checkpoint.md)
