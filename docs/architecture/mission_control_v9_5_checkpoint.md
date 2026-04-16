# Mission Control V9.5 — bounded live coordinator propagation through bridge-facing degraded advisory (checkpoint)

## Strategic question

Can the **live** ``assign_named_navigation`` path preserve ``degraded_passage_advisory`` when that
observation originates from the **bridge** named-navigation seam (V9.4 transport + JSON field),
without changing dispatch, blocked/failure semantics, or turning degraded into control or
classification?

## V9.5 answer (this milestone)

Yes. The chain **bridge service → MissionClient JSON parse → MissionTools normalize (V9.4) →
facade ``handle_command`` → coordinator V9.2 merge** already carried the payload once the bridge
surfaced it; the remaining gap for **automated proof** was in-process **executor colocation**:
``MissionClient`` defaults to a dedicated node spun only by ``spin_until_future_complete``, which
does not run a separate ``mission_bridge_node`` service server unless that server shares an
executor with the client.

**Minimal seam:** optional ``client_rcl_node`` on ``MissionClient`` / ``MissionAgentFacade.with_ros``
/ ``assign_named_navigation`` reuses an existing ``rclpy`` ``Node`` that is already on the same
spinning ``MultiThreadedExecutor`` as ``mission_bridge_node``. Production call sites omit it
(default unchanged).

## Scope

- Optional keyword-only parameters (embedding / bounded tests).
- One ROS test module: degraded ingest + live service + ``assign_named_navigation`` + patched
  Nav2 pose submit + patched terminal wait (no Nav2 stack, no broader launch).

## Tests

```bash
cd /path/to/HDNS
. install/setup.bash   # after colcon build
python3 -m pytest tests/test_coordinator_bridge_degraded_advisory_v95_ros.py -q --tb=short
```

## Related

- [mission_control_v9_4_checkpoint.md](mission_control_v9_4_checkpoint.md)
- [mission_control_v9_2_checkpoint.md](mission_control_v9_2_checkpoint.md)
