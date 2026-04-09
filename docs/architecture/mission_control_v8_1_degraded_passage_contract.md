# Mission Control V8.1 — `degraded_passage` contract freeze

## Purpose

V8.1 defines one second semantic fact type, `degraded_passage`, with the same bounded
contract discipline used for `blocked_passage`.

This milestone is contract-only:

- no launch integration
- no bridge/coordinator behavior changes
- no new execution consequences

## Why `degraded_passage` is distinct

`blocked_passage` means "do not choose this named location as target at this decision step"
under the existing advisory gate.

`degraded_passage` means "the location is still potentially traversable, but conditions are
degraded and this is advisory metadata only."

V8.1 does not grant authority to block, stop, replan, retry, or trigger alternates from
`degraded_passage`.

## Frozen record shape (V8.1)

Record contract (logical, transport-neutral):

| Field | Type / constraint | Meaning |
|-------|-------------------|---------|
| `schema_version` | string, exactly `"v8.1"` | Parser gate for this fact contract. |
| `belief_id` | UUID v4 string | Unique assertion instance id. |
| `fact_type` | string, exactly `"degraded_passage"` | Frozen fact type literal for V8.1. |
| `source_robot_id` | non-empty string | Mission-layer source id. |
| `timestamp_utc` | RFC3339 string | Assertion timestamp at source. |
| `confidence` | float in `[0.0, 1.0]` | Source confidence (ordinal self-report). |
| `location_ref` | non-empty string | Mission named-location identifier. |
| `provenance` | object with required keys `sensor_class` (non-empty string) and `observation_id` (UUID v4 string) | Bounded evidence summary. |
| `ttl_sec` | float `> 0.0` | Active window duration in seconds. |
| `verification_status` | string, exactly `"unverified"` | Frozen status in this milestone. |
| `degradation_class` | string enum: `slow_zone`, `narrow_clearance`, `intermittent_obstacle`, `surface_uncertain` | Bounded degradation category. |
| `recommended_speed_factor` | float in `(0.0, 1.0]` | Advisory speed scalar hint only. |

## Reuse constraints from existing discipline

- Deterministic code remains owner of parsing, reject classification, assembly, validation, and
  store admission.
- Active/inactive semantics remain bounded by explicit `timestamp_utc + ttl_sec + skew` policy.
- Duplicate behavior remains by `belief_id` identity (ignore duplicate ids).
- No geometric truth transfer: `location_ref` remains a named-location semantic anchor.

## Policy meaning (frozen for V8.1)

`degraded_passage` has advisory semantics only.

For V8.1, this fact type may be stored and queried by future milestones, but it must not:

- block target selection directly
- force hard-stop outcomes
- trigger alternate attempts
- mutate maps, costmaps, or TF
- introduce retries/recovery frameworks

## Explicit rejection examples (contract intent)

Reject if any of the following occurs:

- `fact_type != "degraded_passage"`
- missing required field or extra unknown top-level field
- `recommended_speed_factor <= 0.0` or `> 1.0`
- `degradation_class` not in allowlisted enum
- malformed UUIDs or non-RFC3339 timestamp
- inactive at evaluation time by TTL policy

## Out of scope in V8.1

- runtime handoff/bridge/coordinator integration
- second new fact type
- policy wiring to mission outcomes
- taxonomy expansion beyond `degradation_class` enum above
- redesign of store semantics, TTL design, or duplicate handling model

## Related

- [mission_control_v3_0_1_semantic_fact_exchange.md](mission_control_v3_0_1_semantic_fact_exchange.md)
- [mission_control_v7_3_checkpoint.md](mission_control_v7_3_checkpoint.md)
