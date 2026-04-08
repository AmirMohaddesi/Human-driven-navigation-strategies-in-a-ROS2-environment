# Mission Control V4.2 — Bounded launch proof for mission-bridge advisory seam (checkpoint)

## Strategic question

Can the **V4.1** optional mission-bridge advisory seam (frozen V3.0.1 transport → ingest → named-location gate) survive a **launch-based** runtime topology: a **live** `mission_bridge_node` process, a separate publisher of transport JSON, and service calls to `NavigateToNamedLocation`—without changing bridge services, schemas, or advisory semantics?

## Purpose

**Runtime-topology proof** for the bridge seam only (not new semantics). V4.1 proved the same logic in-process; V4.2 proves it under `ros2 launch` + DDS.

## Audit summary

- **Gap:** V4.1 tests call `MissionBridgeNode.navigate_to_named_location` inside a multi-threaded executor; they do not start `mission_bridge_node` as a **separate launched process**.
- **Blocker removed:** `mission_bridge_node`’s `main()` normally waits up to **120s** for Nav2 action servers. For headless advisory proof, a **default-on** parameter `wait_for_nav2_action_servers` (skip wait when `false`) keeps production behavior unchanged while allowing a minimal launch.
- **Smallest topology:** One launch file → one bridge node (advisory topic + allowlist + skip Nav2 wait). Test subprocess starts launch; orchestrator **publishes** one `std_msgs/String` JSON on that topic and **calls** `/navigate_to_named_location` (same public bridge API as production).

## Already proven (V4.1 and earlier)

- Transport encode/ingest, advisory message text, default-off bridge, in-process bridge tests.

## Newly proven by V4.2

- `ros2 launch multi_robot_mission_stack mission_bridge_advisory_v42.launch.py` runs a bridge process with advisory ingest on `/semantic/blocked_passage_v42_launch`.
- Published frozen JSON → service `NavigateToNamedLocation(robot1, base)` → `failed` + `navigation target blocked by peer belief` + empty goal.
- Control: belief for another `location_ref` does not block `base`.

## Intentionally not proven

- Full Nav2 stack, semantic handoff node on same launch, real LLM, pose-level bridge advisory, second fact type.

## Build / source

After pulling, install the new launch file:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ws
colcon build --packages-select multi_robot_mission_stack
source install/setup.bash
```

## Manual launch

```bash
ros2 launch multi_robot_mission_stack mission_bridge_advisory_v42.launch.py
```

Publish a valid V3.0.1 JSON record (e.g. `ros2 topic pub --once ...`) on `/semantic/blocked_passage_v42_launch`, then:

```bash
ros2 service call /navigate_to_named_location multi_robot_mission_stack_interfaces/srv/NavigateToNamedLocation "{robot_id: 'robot1', location_name: 'base'}"
```

Expect blocked response when the record’s `location_ref` matches `base` and is still active.

## Automated test

```bash
cd /path/to/HDNS
source /opt/ros/humble/setup.bash
source /path/to/ws/install/setup.bash
python3 -m pytest tests/test_mission_bridge_advisory_launch_v42_ros.py -v
```

## Related

- [mission_control_v4_1_checkpoint.md](mission_control_v4_1_checkpoint.md)
- [mission_control_v3_9_checkpoint.md](mission_control_v3_9_checkpoint.md)
