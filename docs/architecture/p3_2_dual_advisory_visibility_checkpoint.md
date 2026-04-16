# P3.2 — terminal latest-state board for dual-advisory transport visibility (checkpoint)

## Strategic question

Can a reviewer instantly see the latest blocked/degraded transport state and recent topic activity
without reading long scrollback, while keeping the runtime surface strictly visibility-only?

## Answer (this milestone)

Yes. `p3_2_dual_advisory_visibility.launch.py` starts the same P1.1 bridge transport setup plus
`p3_2_dual_advisory_board`, a passive `rclpy` watcher that subscribes only to:

- `/semantic/blocked_passage_p1_1`
- `/semantic/degraded_passage_p1_1`

The board prints compact terminal markers:

- `[P3.2_ROW] lane=blocked ...`
- `[P3.2_ROW] lane=degraded ...`
- `[P3.2_EVT] ...` recent events (bounded in memory)

Each row includes latest receive time, status, digest fields (`fact_type`, `location_ref`,
`belief_id`), raw length, and counters (`valid`, `malformed`, `empty`, `non_object`).

## Honesty / scope

- Visibility-only runtime observer; no control authority was added.
- No service clients, no action clients, no control publishers in the watcher.
- No new semantic meaning was introduced.
- `shared_space_caution` remains out of runtime scope for this surface.
- No browser UI, rosbridge, backend orchestration, persistence, or auth were introduced.

## How to launch

```bash
cd /path/to/HDNS
source install/setup.bash
ros2 launch multi_robot_mission_stack p3_2_dual_advisory_visibility.launch.py
```

## Manual verification (quick)

In another sourced terminal:

```bash
cd /path/to/HDNS
source install/setup.bash
python3 scripts/p1_1_print_blocked_advisory_wire.py | python3 scripts/p1_1_publish_std_string_once.py /semantic/blocked_passage_p1_1
python3 scripts/p1_1_print_degraded_advisory_wire.py | python3 scripts/p1_1_publish_std_string_once.py /semantic/degraded_passage_p1_1
```

Then publish malformed payload:

```bash
ros2 topic pub -1 /semantic/blocked_passage_p1_1 std_msgs/msg/String "{data: '{not-json'}"
```

Expected: board rows remain live, malformed counter increments for blocked, and watcher stays up.

## Verification (automated)

```bash
cd /path/to/HDNS
source install/setup.bash
python3 -m pytest tests/test_p3_2_dual_advisory_visibility_ros.py -q --tb=short
```

## Related

- [p3_1_dual_advisory_visibility_checkpoint.md](p3_1_dual_advisory_visibility_checkpoint.md)
- [p3_3_p3_2_visibility_demo_wrapper_checkpoint.md](p3_3_p3_2_visibility_demo_wrapper_checkpoint.md)
- [p1_1_headless_dual_advisory_bridge_demo_checkpoint.md](p1_1_headless_dual_advisory_bridge_demo_checkpoint.md)
