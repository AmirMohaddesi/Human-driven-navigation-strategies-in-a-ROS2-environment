# Mission Control V5.1 — Bounded advisory failure classification (checkpoint)

## Strategic question

When named navigation **fails without a goal** because of the **existing** advisory `blocked_passage` outcome (frozen `outcome` + `reason_code` on the navigate result dict), can orchestration record **one allowlisted machine-readable field** on sequence/parallel/coordinator leg results **without** changing goals sent, store semantics, or policy?

## Purpose (V5 family)

First **execution-relevance** artifact: **classification only**—no retries, no fallback targets, no new gates.

## Detection rule

`navigate_failure_kind` is set to the allowlisted token **`advisory_blocked_passage`** only when the navigate dict matches:

- `outcome == navigation_target_blocked` (`BLOCKED_OUTCOME_VALUE`)
- `reason_code == blocked_passage_peer_belief` (`BLOCKED_REASON_CODE`)

as produced by `MissionTools._blocked_by_peer_belief_response` / `make_blocked_by_peer_belief_outcome`.

**Bridge-normalized** responses that omit these fields do **not** get this classification (V5.1 does not claim bridge parity).

## Additive fields

| Location | Field |
|----------|--------|
| `sequence_utils` step record | `navigate_failure_kind` (optional) |
| `parallel_utils` step record | `navigate_failure_kind` (optional) |
| `assign_named_navigation` return | `navigate_failure_kind` (optional) |

Legacy `error` / `message` behavior is **unchanged**.

## Module

`multi_robot_mission_stack.agent.navigate_failure_classification_v51`

## Tests (local, no ROS)

```bash
cd /path/to/HDNS
python3 -m pytest \
  tests/test_navigate_failure_classification_v51.py \
  tests/test_sequence_utils.py \
  tests/test_parallel_utils.py \
  tests/test_coordinator.py::test_v51_assign_navigate_advisory_blocked_sets_navigate_failure_kind \
  tests/test_coordinator.py::test_assign_navigate_fails \
  -q
```

## Related

- [mission_control_v4_3_checkpoint.md](mission_control_v4_3_checkpoint.md) (semantic → bridge advisory; different concern)
