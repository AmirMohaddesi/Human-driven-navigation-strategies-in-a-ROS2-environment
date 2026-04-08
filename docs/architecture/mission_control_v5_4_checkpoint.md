# Mission Control V5.4 — Bounded end-to-end reporting on handoff→bridge→coordinator (checkpoint)

## Strategic question

Can a **valid semantic handoff** (deterministic fake adapter), mirrored onto the frozen V3.0.1
transport topic and ingested by **mission_bridge_node**, lead the **coordinator**
``assign_named_navigation`` path to report ``navigate_failure_kind == advisory_blocked_passage``?

## Purpose

**End-to-end launched reporting parity only** — compose proven V4.3 transport semantics with proven
V5.3 coordinator classification; no new control or contracts.

## Smallest topology (audit)

- **Launch:** Existing ``semantic_handoff_mission_bridge_v43.launch.py`` only:
  - ``semantic-production-handoff-v35`` with ``mirror_ingested_record_to_transport`` and
    ``mirror_transport_topic`` = ``/semantic/blocked_passage_v43_e2e``
  - ``mission_bridge_node`` with ``advisory_blocked_passage_transport_topic`` on the **same** topic
- **No new nodes, topics, or launch files** for V5.4.
- **Test process:** handoff ``ProduceSemanticBlockedPassageV35`` + ``assign_named_navigation`` (same
  envelope and timing pattern as V4.3 E2E).

## Proof

| Case | Steps | Expectation |
|------|--------|-------------|
| Blocked | Valid handoff for ``base`` → brief settle → ``assign_named_navigation(robot1, base)`` | Failed navigate, peer-belief message, ``navigate_failure_kind`` set |
| Control | Invalid JSON handoff → ``assign_named_navigation`` | No advisory message, no ``navigate_failure_kind`` |

## Tests

**Launch / runtime:**

```bash
cd /path/to/HDNS
python3 -m pytest tests/test_semantic_handoff_bridge_coordinator_v54_ros.py -q --tb=short
```

**Local regression (no handoff launch):**

```bash
python3 -m pytest tests/test_coordinator_bridge_reporting_launch_v53_ros.py \
  tests/test_semantic_handoff_bridge_e2e_v43_ros.py -q --tb=short
```

## Related

- [mission_control_v4_3_checkpoint.md](mission_control_v4_3_checkpoint.md) (handoff→bridge E2E)
- [mission_control_v5_3_checkpoint.md](mission_control_v5_3_checkpoint.md) (bridge→coordinator launch)
