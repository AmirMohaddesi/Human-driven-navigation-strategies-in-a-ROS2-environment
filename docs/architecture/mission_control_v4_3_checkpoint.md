# Mission Control V4.3 — Bounded end-to-end launch: semantic handoff → bridge advisory block (checkpoint)

## Strategic question

Can the **already-proven** launched semantic handoff (V3.9 mirror path) and **already-proven** launched mission-bridge advisory seam (V4.2) run in **one** bounded topology so that a **successful handoff** leads to the **same** advisory blocked `NavigateToNamedLocation` outcome on the **live** bridge—via the **unchanged** V3.0.1 transport JSON only?

## Purpose

**Compose** two proven launch pieces into one **end-to-end runtime** proof. No new semantics, APIs, or mission execution claims.

## Audit summary

- **V3.9** launch: handoff + witness share a topic; mirror publishes after `ingest_stored`.
- **V4.2** launch: bridge alone subscribes to advisory topic and gates named navigation.
- **Smallest composition:** Replace witness with **bridge** on the **same** topic as handoff mirror—two nodes only: `semantic-production-handoff-v35` + `mission_bridge_node`. Reuse existing mirror and advisory params; isolated topic `/semantic/blocked_passage_v43_e2e`.

## Already proven (before V4.3)

- Handoff → mirror → witness (V3.9); handoff core unchanged.
- Bridge advisory ingest + service block (V4.1/V4.2).

## Newly proven by V4.3

- `ros2 launch multi_robot_mission_stack semantic_handoff_mission_bridge_v43.launch.py` runs handoff + bridge together.
- Orchestrator: valid bounded envelope (fake adapter, wall clocks) → `ingest_stored` → `NavigateToNamedLocation(robot1, base)` → `failed` + peer-belief message.
- Control: invalid handoff → no mirror payload for a stored belief → navigate `base` **not** advisory-blocked.

## Intentionally not proven

- Real LLM on this graph, Nav2 success, witness + bridge simultaneously, second fact type, product hardening.

## Build / source

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ws
colcon build --packages-select multi_robot_mission_stack_interfaces multi_robot_mission_stack
source install/setup.bash
```

## Launch

```bash
ros2 launch multi_robot_mission_stack semantic_handoff_mission_bridge_v43.launch.py
```

Then call `produce_semantic_blocked_passage_v35` with a wall-aligned envelope (`use_deterministic_fake_adapter: true`), then `/navigate_to_named_location` for `base`.

## Automated test

```bash
cd /path/to/HDNS
source /opt/ros/humble/setup.bash
source /path/to/ws/install/setup.bash
python3 -m pytest tests/test_semantic_handoff_bridge_e2e_v43_ros.py -v
```

## Related

- [mission_control_v4_2_checkpoint.md](mission_control_v4_2_checkpoint.md)
- [mission_control_v3_9_checkpoint.md](mission_control_v3_9_checkpoint.md)
