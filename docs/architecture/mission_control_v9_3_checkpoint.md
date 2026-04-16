# Mission Control V9.3 — bounded launch/runtime proof for degraded advisory mission propagation (checkpoint)

## Strategic question

Can the additive `degraded_passage_advisory` observation be exercised on a **live ROS ingest path**
and still appear on **coordinator-facing** leg-shaped output, without changing dispatch, blocked/failure
semantics, or turning degraded into control or classification?

## V9.3 answer (this milestone)

Yes, on the **smallest composed runtime path** (no bridge change):

1. `DegradedPassageTransportReceiverNode` + `std_msgs/String` JSON on a dedicated test topic (same
   pattern as [mission_control_v8_9_checkpoint.md](mission_control_v8_9_checkpoint.md)).
2. A **shared** `DegradedPassageBeliefStore` injected into both the receiver and `MissionTools`.
3. `MissionAgentFacade.handle_command(navigate, now_utc=...)` (same graph/tools path the coordinator
   uses for navigate via facade).
4. Coordinator merge helper `_leg_dict_with_degraded_advisory_from_navigate` (same logic as
   `assign_named_navigation` after V9.2).

**Positive:** publish valid degraded record → ingest → navigate dispatches once → advisory on
`nav` → preserved on merged leg dict.

**Control:** ROS ingest fills a receiver-only store while `MissionTools` is built **without**
`degraded_passage_store` — navigate still dispatches once; `degraded_passage_advisory` is absent
from both `nav` and coordinator-shaped output (mission stack does not observe degraded).

## Scope

- One new multiprocess-style test module; no bridge, policy, `navigate_failure_kind`, or launch
  graph expansion beyond what the test spins in-process.

## Tests

```bash
cd /path/to/HDNS
python3 -m pytest tests/test_degraded_passage_mission_runtime_v93.py -q --tb=short
```

## Related

- [mission_control_v8_9_checkpoint.md](mission_control_v8_9_checkpoint.md)
- [mission_control_v9_2_checkpoint.md](mission_control_v9_2_checkpoint.md)
