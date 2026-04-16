# Mission Control V10.1 — bounded bridge execution-reporting enrichment for degraded advisory (checkpoint)

## Strategic question

When named navigation **actually dispatches** a Nav2 goal (non-empty `goal_id` on the service
response), can the bridge’s structured **`navigate_named_result`** log carry **bounded**
degraded-advisory execution context **without** changing service responses, refusal behavior,
failure classification, or Nav2 control authority?

## V10.1 answer (this milestone)

Yes, **logs only**: the existing `navigate_named_result` JSON line gains **optional** additive
fields when **both** are true:

1. `goal_id` is non-empty after `navigate_to_named_location`, and  
2. `degraded_passage_advisory` is a dict with **`has_active` is exactly `True`**.

Bounded fields (no full advisory JSON in logs):

- `degraded_advisory_has_active` — always `true` when this branch applies  
- `degraded_advisory_entry_count` — `min(max(len(entries), len(active_belief_ids)), 99)`  
- `degraded_advisory_belief_ids` — up to **5** belief ids from entries, then `active_belief_ids`  
- `degraded_advisory_belief_ids_truncated` — `true` when source lists exceed 5  

**Pattern chosen:** extend **`navigate_named_result`** additively (one line, one event name) so
grep/journald consumers keep a single stream; no second event type.

## Scope

- `mission_bridge_node.py` helper + handler log merge only.

## Tests

```bash
cd /path/to/HDNS
. install/setup.bash
python3 -m pytest tests/test_mission_bridge_degraded_execution_log_v101.py -q --tb=short
```

## Related

- V10 family charter (manager): execution-adjacent reporting first; no blocking degraded.
