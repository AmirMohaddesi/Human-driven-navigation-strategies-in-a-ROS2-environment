# Mission Control V9.1 — bounded non-blocking degraded_passage mission observation (checkpoint)

## Strategic question (V9 family)

Can `degraded_passage` gain one narrowly defined, **non-blocking** mission-layer meaning that
stays **advisory-only**, remains **distinct** from `blocked_passage`, and does not redesign policy
or execution authority?

## V9.1 answer (this milestone)

Yes, at the **MissionTools** seam only: when `degraded_passage_store` is configured,
`navigate_to_named_location` attaches a single additive key,
`degraded_passage_advisory` (see `DEGRADED_PASSAGE_ADVISORY_RESPONSE_KEY` in `mission_tools.py`),
whose value is `DegradedPassageAdvisoryReport.to_dict()` for the requested named location.

- **Non-blocking:** active degraded beliefs do **not** prevent dispatch to the mission client (tests
  assert `navigate_to_named_location` on the client is invoked exactly once when not blocked).
- **Distinct from blocked:** `blocked_passage` still exclusively controls the advisory block gate;
  degraded never sets `outcome` / `reason_code` / blocked failure message.
- **Clock:** `now_utc` is required when `degraded_passage_store` is configured (same explicit-clock
  discipline as blocked store).

## Scope

- `MissionTools` only — no bridge, coordinator, ROS transport, or launch changes in V9.1.
- No hard-stop, refusal, or alternate-target behavior for degraded.

## Tests

```bash
cd /path/to/HDNS
python3 -m pytest tests/test_mission_tools_degraded_passage_advisory_v91.py -q --tb=short
```

## Related

- [mission_control_v8_9_checkpoint.md](mission_control_v8_9_checkpoint.md)
- [mission_control_v8_1_degraded_passage_contract.md](mission_control_v8_1_degraded_passage_contract.md)
