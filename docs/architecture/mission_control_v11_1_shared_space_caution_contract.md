# Mission Control V11.1 — `shared_space_caution` contract freeze

## Purpose

V11.1 defines one third semantic fact type, `shared_space_caution`, with the same bounded
contract discipline used for `blocked_passage` and `degraded_passage`.

This milestone is contract-only:

- no launch integration
- no bridge/coordinator or mission-layer behavior changes
- no transport, store, or production validator code

## Strategic question

Can we freeze a third `fact_type` literal whose meaning is neither refusal-capable blockage nor
passage-quality degradation, and document it with the same strict contract discipline—including
explicit reject examples—without any runtime integration?

## Why `shared_space_caution` is distinct

`blocked_passage` means, under the existing advisory gate, that robot B must not choose the
matching named location as the navigation target for that decision step.

`degraded_passage` means the named location remains potentially traversable, but passage-quality
metadata (bounded `degradation_class` and optional advisory speed scalar) describes degraded
conditions.

`shared_space_caution` means short-lived **shared occupancy / coexistence** context at the named
location: people or manual operations may be present in the same volume. It does **not** assert
that the route is impassable, narrowed, surface-uncertain, or that any speed scalar applies.

V11.1 does not grant authority to refuse targets, stop motion, replan, retry, trigger alternates,
or mutate classical geometry from `shared_space_caution`.

## Frozen record shape (V11.1)

Record contract (logical, transport-neutral):

| Field | Type / constraint | Meaning |
|-------|-------------------|---------|
| `schema_version` | string, exactly `"v11.1"` | Parser gate for this fact contract. |
| `belief_id` | UUID v4 string | Unique assertion instance id. |
| `fact_type` | string, exactly `"shared_space_caution"` | Frozen fact type literal for V11.1. |
| `source_robot_id` | non-empty string | Mission-layer source id. |
| `timestamp_utc` | RFC3339 string | Assertion timestamp at source. |
| `confidence` | float in `[0.0, 1.0]` | Source confidence (ordinal self-report). |
| `location_ref` | non-empty string | Mission named-location identifier. |
| `provenance` | object with required keys `sensor_class` (non-empty string) and `observation_id` (UUID v4 string) | Bounded evidence summary (same shape as V8.1). |
| `ttl_sec` | float `> 0.0` | Active window duration in seconds. |
| `verification_status` | string, exactly `"unverified"` | Frozen status in this milestone. |
| `caution_class` | string enum: `pedestrian_activity`, `manual_handling`, `maintenance_presence` | Bounded shared-space category. |

## Reuse constraints from existing discipline

- Deterministic code remains owner of parsing, reject classification, assembly, validation, and
  store admission when this type is wired in a later milestone.
- Active/inactive semantics follow the same **intent** as V3.0.1 / V8.1: evaluation uses
  `timestamp_utc` plus `ttl_sec` plus a small agreed skew; inactive records must not influence
  policy.
- Duplicate handling **intent**: same assertion instance is keyed by `belief_id`; duplicates are
  ignored by identity when a store exists.
- No geometric truth transfer: `location_ref` remains a named-location semantic anchor only.

## Policy meaning (frozen for V11.1)

`shared_space_caution` has **advisory-only** semantics in this milestone.

This fact type must not:

- block or refuse named navigation targets (that remains `blocked_passage` only where proven)
- imply passage degradation, clearance limits, or traversability quality (that remains
  `degraded_passage`)
- carry a speed scalar or any motion command
- force hard-stop outcomes, replanning trees, retries, or alternates
- mutate maps, costmaps, TF, or other classical truth

## What this fact type is not

- Not a statement of blockage or impassability.
- Not a statement of narrow clearance, slow zone, intermittent obstacle, or uncertain surface
  (those belong to `degraded_passage` when used).
- Not a policy or access-control rule, permit, or compliance record.
- Not calibrated probability, geometric ground truth, or natural-language narrative.

## Explicit rejection examples (contract intent)

Reject if any of the following occurs:

- `fact_type != "shared_space_caution"`
- `schema_version != "v11.1"`
- missing required field or extra unknown top-level field
- `caution_class` not in the allowlisted enum above
- `confidence` outside `[0.0, 1.0]` or `ttl_sec <= 0.0`
- `verification_status != "unverified"`
- malformed `belief_id` or `provenance.observation_id` (must be UUID v4)
- `provenance` missing required keys, extra keys, empty `sensor_class`, or non-RFC3339
  `timestamp_utc` per fixture validation rules in the V11.1 docsync test

## Out of scope in V11.1

- runtime, bridge, coordinator, MissionTools, or mission behavior
- transport envelopes, ROS topics, or production `semantic/` validators
- fourth fact types or generalized semantic frameworks
- taxonomy expansion beyond `caution_class` above
- redesign of store semantics, TTL implementation, or duplicate handling (beyond documenting
  alignment intent)

## Related

- [mission_control_v11_7_local_proof_bundle_closure.md](mission_control_v11_7_local_proof_bundle_closure.md) — local proof line closure map (V11.1–V11.7)
- [mission_control_v3_0_1_semantic_fact_exchange.md](mission_control_v3_0_1_semantic_fact_exchange.md)
- [mission_control_v8_1_degraded_passage_contract.md](mission_control_v8_1_degraded_passage_contract.md)
- [mission_control_v11_1_checkpoint.md](mission_control_v11_1_checkpoint.md)
