# Mission Control V5.3 — Bounded execution reporting launch parity (checkpoint)

## Strategic question

Can the **live** bridge-mediated named-navigation path, together with the **coordinator**
``assign_named_navigation`` entry (facade + ``MissionClient``), exhibit the same additive
``navigate_failure_kind`` classification for advisory ``blocked_passage`` as in unit tests?

## Purpose

**Runtime reporting parity only** — no new control, services, or policy.

## Smallest topology (audit)

- **Chosen:** Reuse **V4.2** ``mission_bridge_advisory_v42.launch.py`` (headless bridge,
  advisory transport on ``/semantic/blocked_passage_v42_launch``, Nav2 wait skipped).
- **Not required for V5.3:** V4.3 handoff → mirror → bridge chain; that path is already
  proven for transport semantics; V5.3 only needs **orchestration** to consume the **same**
  bridge service responses the V4.2 tests already exercise.

## Proof

| Case | Expectation |
|------|-------------|
| Publish blocked record for ``base``, ``assign_named_navigation(robot1, base)`` | ``navigate_failure_kind == advisory_blocked_passage`` |
| Publish blocked record for ``other_hall`` only, navigate ``base`` | No ``navigate_failure_kind``; message ≠ peer-belief line |

Classification logic remains ``navigate_failure_classification_v51.navigate_failure_kind``
(V5.2 wire-normalized branch).

## Tests

**Launch / runtime (requires ROS 2 + sourced workspace):**

```bash
cd /path/to/HDNS
python3 -m pytest tests/test_coordinator_bridge_reporting_launch_v53_ros.py -q --tb=short
```

**Local regression (no launch):**

```bash
python3 -m pytest tests/test_navigate_failure_classification_v51.py \
  tests/test_coordinator.py::test_v52_assign_navigate_bridge_wire_shape_sets_navigate_failure_kind -q
```

## Related

- [mission_control_v5_1_checkpoint.md](mission_control_v5_1_checkpoint.md) (classification field)
- [mission_control_v4_2_checkpoint.md](mission_control_v4_2_checkpoint.md) (same launch topology)
