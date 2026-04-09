# Mission Control V7.1 — One predeclared alternate named location per sequential leg (checkpoint)

## Strategic question

If a sequential leg’s primary named navigation returns advisory `blocked_passage`
(`navigate_failure_kind == advisory_blocked_passage`), can the coordinator issue **exactly one**
follow-up navigate to a **predeclared** `alternate_location_name` (allowlisted, distinct from the
primary), without model-chosen goals or replanning?

## Purpose (V7 family)

First **bounded fallback** milestone: **one** optional alternate per step, **one** extra navigate,
**advisory-only** trigger.

## V6.1 ↔ V7.1 interaction (frozen rule)

1. Run **primary** `assign_named_navigation`.
2. If primary is advisory-blocked **and** a valid `alternate_location_name` is declared, run **one**
   alternate navigate (same `robot_id`).
3. The **effective** leg result for success/failure, `step_outcome`, and **V6.1 hard-stop**
   evaluation is the **alternate** result when an alternate was attempted; otherwise the primary
   result.

So hard-stop is decided **after** any alternate attempt, on the effective outcome.

## Validation

- Optional `alternate_location_name` per step (coordinator sequential + `normalize_team_named_mission_spec` + spec contract).
- If present and non-empty after strip: must be a string, **≠** `location_name`, and **∈**
  `build_default_policy_config().allowed_locations`.

## Step record (when alternate declared on step)

Additive fields: `alternate_location_name`, `primary_result`, `alternate_attempted`,
`alternate_result`, and `result` = **effective** navigate dict.

Steps **without** an alternate omit those fields (shape unchanged from pre-V7.1).

## Tests (local)

```bash
cd /path/to/HDNS
python3 -m pytest \
  tests/test_coordinator.py::test_v71_advisory_primary_triggers_single_alternate_navigate \
  tests/test_coordinator.py::test_v71_generic_primary_failure_does_not_attempt_alternate \
  tests/test_coordinator.py::test_v71_alternate_success_then_second_leg_runs \
  tests/test_coordinator.py::test_v71_hard_stop_applies_to_effective_leg_after_alternate_still_advisory \
  tests/test_coordinator.py::test_normalize_team_named_mission_spec_alternate_ok \
  -q --tb=short
```

## Out of scope (V7.1)

Multiple alternates, fallback trees, retries, parallel/in-process `sequence_utils`, bridge `.srv`,
store semantics, launch expansion, LLM-chosen targets.

## Related

- [mission_control_v6_1_checkpoint.md](mission_control_v6_1_checkpoint.md) (hard stop)
