# Mission Control V9.4 — bounded bridge-facing degraded advisory propagation (checkpoint)

## Strategic question

Can the existing additive `degraded_passage_advisory` observation surface on the **bridge-facing**
named-navigation path (service + internal dict), without changing blocked/failure semantics,
dispatch authority, or turning degraded into control or classification?

## V9.4 answer (this milestone)

Yes:

1. **Optional bridge ingest** mirrors V4.1 blocked transport: parameters
   `advisory_degraded_passage_transport_topic` and
   `advisory_degraded_passage_allowed_source_robot_ids` wire `std_msgs/String` JSON through
   `ingest_degraded_passage_transport_payload` into a `DegradedPassageBeliefStore` owned by the
   bridge.
2. **`MissionBridgeNode.navigate_to_named_location`** attaches
   `degraded_passage_advisory` (same key as MissionTools / `DEGRADED_PASSAGE_ADVISORY_RESPONSE_KEY`)
   via `query_advisory_degraded_passage` on **every** named-nav outcome path when that store
   exists — observation only; blocked gate and Nav2 dispatch unchanged.
3. **`NavigateToNamedLocation` service** carries one additive response field
   `degraded_passage_advisory_json` (compact JSON, empty when no advisory dict is attached).
4. **`MissionClient`** parses non-empty JSON into the same dict key for `MissionTools`.
5. **`MissionTools._normalize_goal_response`** preserves that optional key so higher layers
   (coordinator, V9.2 merge) keep working unchanged when the bridge forwards observation.

## Scope

- One srv response field + bridge + client + normalize only.
- No `navigate_failure_kind` changes, no new refusal behavior for degraded.

## Tests

Rebuild interfaces after `.srv` change, then:

```bash
cd /path/to/HDNS
colcon build --packages-select multi_robot_mission_stack_interfaces multi_robot_mission_stack
source install/setup.bash
python3 -m pytest tests/test_mission_bridge_degraded_advisory_v94.py -q --tb=short
```

## Related

- [mission_control_v9_3_checkpoint.md](mission_control_v9_3_checkpoint.md)
- [mission_control_v9_2_checkpoint.md](mission_control_v9_2_checkpoint.md)
- [mission_control_v4_1_checkpoint.md](mission_control_v4_1_checkpoint.md) (blocked advisory transport pattern)
