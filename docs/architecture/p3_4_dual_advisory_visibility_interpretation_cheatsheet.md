# P3.4 — dual-advisory visibility interpretation cheatsheet (static)

## Purpose

This is a reading guide for the existing P3 runtime surfaces:

- P3.1 digest stream (`[P3.1] ...`)
- P3.2 latest-state board (`[P3.2_ROW] ...`, `[P3.2_EVT] ...`)

It does **not** change runtime behavior. It only explains how to read already-proven output in
plain language.

## Scope / honesty

- Visibility-only interpretation aid for existing outputs.
- No control authority, no command path, no service/action usage.
- No new semantic meaning; this guide only explains existing fields and statuses.
- `shared_space_caution` remains out of runtime scope for this surface.

## Quick marker guide

- `[P3.1]`: one event digest line per received message.
- `[P3.2_ROW]`: current latest-state snapshot for one topic lane (`blocked` or `degraded`).
- `[P3.2_EVT]`: recent event log entry (newest first in P3.2 board output).

## Representative snippets and plain-language meaning

### P3.1 digest line (valid blocked message)

```text
[P3.1] recv=2026-04-16T19:21:05.117Z topic=/semantic/blocked_passage_p1_1 fact_type='blocked_passage' location_ref='base' belief_id='7f94...' raw_len=312 raw_head='{"schema_version":"3.0.1",...}'
```

If you see this, it means:

- a message arrived on the blocked advisory transport topic;
- JSON parsing succeeded and the core fields were extracted;
- this is observation only (the line does not command motion or cancel anything).

### P3.2 blocked row (latest state)

```text
[P3.2_ROW] lane=blocked topic=/semantic/blocked_passage_p1_1 last_recv=2026-04-16T19:21:05.117Z status=VALID fact_type='blocked_passage' location_ref='base' belief_id='7f94...' raw_len=312 counts(valid=1 malformed=0 empty=0 non_object=0)
```

If you see this, it means:

- the latest blocked-topic payload was valid JSON object payload;
- the blocked lane currently points at that latest parsed fact;
- counters are cumulative for the current node runtime (since launch).

### P3.2 degraded row (latest state)

```text
[P3.2_ROW] lane=degraded topic=/semantic/degraded_passage_p1_1 last_recv=2026-04-16T19:21:08.403Z status=VALID fact_type='degraded_passage' location_ref='base' belief_id='3c21...' raw_len=338 counts(valid=1 malformed=0 empty=0 non_object=0)
```

If you see this, it means:

- the latest degraded-topic payload was valid and now occupies the degraded lane snapshot;
- the degraded lane status/counters are independent from the blocked lane counters.

### P3.2 malformed event line

```text
[P3.2_EVT] recv=2026-04-16T19:21:11.922Z topic=/semantic/blocked_passage_p1_1 status=UNPARSEABLE_JSON err='Expecting property name enclosed in double quotes: line 1 column 2 (char 1)' raw_head='{not-json'
```

If you see this, it means:

- the watcher received a payload that was not parseable JSON;
- the watcher handled it safely (event logged) rather than crashing;
- the blocked `malformed` counter should increase on subsequent `[P3.2_ROW]` lines.

## Standard P3.3 walkthrough (blocked -> degraded -> malformed)

Use `scripts/p3_3_operator_p3_2_visibility_demo.sh` with P3.2 launch running.

1. **Blocked step**
   - P3.1: one digest line with `fact_type='blocked_passage'`.
   - P3.2 blocked row: `status=VALID`, blocked `valid` counter increments.

2. **Degraded step**
   - P3.1: one digest line with `fact_type='degraded_passage'`.
   - P3.2 degraded row: `status=VALID`, degraded `valid` counter increments.

3. **Malformed step**
   - P3.1: one digest line with `UNPARSEABLE_JSON`.
   - P3.2 events: one `[P3.2_EVT]` with `status=UNPARSEABLE_JSON`.
   - P3.2 blocked row: `malformed` counter increments; board stays alive.

## Practical read pattern for operators/reviewers

- First, confirm the two topic lanes are present in `[P3.2_ROW]`.
- Then, watch `status` + counters (`valid`, `malformed`, `empty`, `non_object`) per lane.
- Use `[P3.2_EVT]` for immediate event detail and `[P3.1]` for per-message digest context.
- Treat all of this as runtime observation only, not command authority.

## Related

- [p3_1_dual_advisory_visibility_checkpoint.md](p3_1_dual_advisory_visibility_checkpoint.md)
- [p3_2_dual_advisory_visibility_checkpoint.md](p3_2_dual_advisory_visibility_checkpoint.md)
- [p3_3_p3_2_visibility_demo_wrapper_checkpoint.md](p3_3_p3_2_visibility_demo_wrapper_checkpoint.md)
