# P3.1 — launch-bundled read-only dual-advisory transport visibility (checkpoint)

## Strategic question

With the existing P1.1 bridge running, can a reviewer **continuously observe** blocked-ingest and
degraded-ingest `std_msgs/String` JSON traffic on the two configured topics in **one**
terminal-oriented surface, with **zero control actions** and **no** new semantic types?

## Answer (this milestone)

Yes. **`p3_1_dual_advisory_visibility.launch.py`** starts the same headless **P1.1** bridge
parameters as `mission_bridge_headless_dual_advisory_p11.launch.py` plus
**`p3_1_dual_advisory_visibility`**, a passive **rclpy** node that **subscribes only** to:

- `/semantic/blocked_passage_p1_1`
- `/semantic/degraded_passage_p1_1`

Each message prints a **`[P3.1]`** stdout digest (receipt time, topic, parsed `fact_type` /
`location_ref` / `belief_id`, truncated raw head). Malformed JSON prints **`UNPARSEABLE_JSON`**
without crashing.

## Honesty / scope

- **Visibility-only:** no buttons, no goals, no service/action clients from this node.
- **Not a mission dashboard:** digest is transport observation, not authority over motion or maps.
- **No new semantics:** same JSON shapes as existing P1.1 lab and tests.
- **`shared_space_caution`** remains **out of scope** for this runtime surface.
- **No** rosbridge, browser live stack, auth, persistence, or coordinator/bridge contract changes.

## How to launch (manual)

```bash
cd /path/to/HDNS
source install/setup.bash
ros2 launch multi_robot_mission_stack p3_1_dual_advisory_visibility.launch.py
```

In another terminal (sourced), publish payloads the same way as the P1.1 checkpoint (e.g.
`ros2 topic pub -1 … std_msgs/msg/String "{data: '…'} "` using JSON from
`scripts/p1_1_print_blocked_advisory_wire.py` / degraded helpers, or follow
`tests/test_mission_bridge_dual_advisory_launch_p11_ros.py` patterns).

You should see **`[P3.1]`** lines interleaved with bridge logs on the launch terminal.

## Verification

Automated (requires `ros2` on PATH and built workspace):

```bash
cd /path/to/HDNS
source install/setup.bash
python3 -m pytest tests/test_p3_1_dual_advisory_visibility_ros.py -q --tb=short
```

## Related

- [p1_1_headless_dual_advisory_bridge_demo_checkpoint.md](p1_1_headless_dual_advisory_bridge_demo_checkpoint.md)
- [p2_5_reviewer_ready_replay_bundle_closure.md](p2_5_reviewer_ready_replay_bundle_closure.md) (offline P2; contrast only)
