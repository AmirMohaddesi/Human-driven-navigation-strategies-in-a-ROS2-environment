# Mission system runbook

## A. Purpose

This document tells developers and operators how to **run**, **smoke-test**, and **debug** the mission stack in this repository: mock path (no ROS), ROS path (bridge + Nav2), CLI entrypoint, and standalone validation scripts. For layered design, see [mission_system_overview.md](mission_system_overview.md).

---

## B. Supported execution surfaces

| Surface | What it uses |
|---------|----------------|
| **Mock CLI** | `mission-agent --mock …` → `MissionAgentFacade.with_mock()` |
| **ROS CLI** | `mission-agent --ros …` → `MissionAgentFacade.with_ros()` |
| **Direct ROS scripts** | `scripts/validate_mission_client_ros.py` (client only), `scripts/validate_mission_agent_facade_ros.py` (facade end-to-end) |

---

## C. Mock mode quickstart

No ROS, no workspace source required (Python env must have `langgraph` and package deps from `setup.py`).

```bash
mission-agent --mock navigate --robot-id robot1 --location-name base
mission-agent --mock query-state --robot-id robot1 --goal-id mock-goal-001
```

From a checkout with `src` on `PYTHONPATH` (or `python -m` from `src/`):

```bash
python -m multi_robot_mission_stack.agent.cli --mock navigate --robot-id robot1 --location-name base
```

**Use when:** exercising CommandAdapter, MissionPolicy, MissionGraph, and MissionAgentFacade without `rclpy` or a running bridge.

---

## D. ROS mode quickstart

**Assumptions:** Ubuntu (or ROS-supported OS), ROS 2 distro sourced, this repo under your colcon workspace `src/`, dependencies (Nav2, TurtleBot3 stacks, etc.) available for the integrated launch.

**Build and source** (from workspace root):

```bash
colcon build --packages-select multi_robot_mission_stack_interfaces multi_robot_mission_stack
source install/setup.bash
```

**Launch** simulation + Nav2 + mission bridge (heavy stack; adjust if you maintain a slimmer launch):

```bash
ros2 launch multi_robot_mission_stack fully_integrated_swarm.launch.py
```

**Service check** (second terminal, same `source`):

```bash
ros2 service list | grep -E '^/(navigate_to_pose|navigate_to_named_location|get_navigation_state|cancel_navigation)$'
```

**CLI examples:**

```bash
mission-agent --ros navigate --robot-id robot1 --location-name base
mission-agent --ros query-state --robot-id robot1 --goal-id "<goal_id>"
```

Use the `goal_id` printed by a successful navigate (JSON field `goal_id`).

---

## E. Direct validation scripts

Run from the **repository root** (scripts add `src/` to `sys.path`; prefer the same shell where `install/setup.bash` is sourced so generated interfaces resolve).

```bash
python3 scripts/validate_mission_client_ros.py
python3 scripts/validate_mission_agent_facade_ros.py
```

| Script | Validates |
|--------|-----------|
| `validate_mission_client_ros.py` | `MissionClient`: `navigate_to_named_location` → `get_navigation_state` |
| `validate_mission_agent_facade_ros.py` | `MissionAgentFacade.with_ros()` with external navigate + query commands |

Nonzero exit on failure; compact JSON lines on stdout.

---

## F. Expected service endpoints

With default bridge node name `mission_bridge_node`:

- `/navigate_to_pose`
- `/navigate_to_named_location`
- `/get_navigation_state`
- `/cancel_navigation`

---

## G. Failure interpretation

| Symptom | Likely failing layer | Next thing to check |
|---------|----------------------|---------------------|
| `ModuleNotFoundError: rclpy` (or interfaces import error) | Environment / workspace | ROS distro sourced? `colcon build` for `multi_robot_mission_stack_interfaces`? Use `--mock` if you only need non-ROS tests. |
| Services missing in `ros2 service list` | Launch / bridge | Is `fully_integrated_swarm` (or your launch) running? Node name still `mission_bridge_node`? |
| JSON `status`/`message` like service not available / timed out | `MissionClient` / DDS | Bridge up? Network/Firewall? Increase load wait; confirm service names match §F. |
| `policy denied:` in JSON | MissionPolicy | `robot_id`, `location_name` in allowlists (`policy_config` / `build_default_policy_config`). |
| `nav_status` / bridge status `rejected` (navigate) | Nav2 / bridge | Namespaces (`robot1_ns`), Nav2 up, map/localization, goal feasible. |
| `status`: `failed` with adapter wording (e.g. unknown target) | CommandAdapter | External command shape: `type` / `target` / required fields. |

---

## H. Current supported commands

Through the **facade and graph**, the supported **external** operations are:

1. **Navigate to named location** — `mission-agent … navigate --robot-id … --location-name …`
2. **Query navigation state** — `mission-agent … query-state --robot-id … --goal-id …`

Other bridge services (e.g. pose navigation, cancel) exist at the ROS layer but are **not** exposed by this CLI or graph in the current release.

---

## Assumptions (summary)

- **Mock path:** Python 3 with package dependencies; no `rclpy`.
- **ROS path:** Built workspace, correct `source`, bridge running with Nav2 for configured robot namespaces; `named_locations.yaml` defines names such as `base` and `test_goal`.
- **CLI name:** `mission-agent` after install; otherwise `python -m multi_robot_mission_stack.agent.cli`.

---

## I. Locked Golden Path Baseline (Live ROS2)

Use this as the canonical acceptance path for today's mission-layer baseline:
- named-location navigate
- query-state
- live ROS2 bridge path

### Canonical rebuild / relaunch / validate sequence

```bash
# 0) mandatory pre-launch cleanup (avoid stale-process contamination)
pkill -f "ros2 launch multi_robot_mission_stack fully_integrated_swarm.launch.py" || true
pkill -f "gzserver|gzclient|rviz2|entity_spawner|lifecycle_manager|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|map_server|amcl|slam_toolbox|robot_state_publisher|initial_pose_publisher|merge_map_node|mission_bridge_node" || true
sleep 1

# 2) rebuild target package only
source /opt/ros/humble/setup.bash
colcon build --packages-select multi_robot_mission_stack

# 3) source overlay
source install/setup.bash

# 4) launch integrated stack (terminal A)
ros2 launch multi_robot_mission_stack fully_integrated_swarm.launch.py
```

If stale processes are not cleared, validation outcomes may be non-reproducible even when mission-layer code is healthy.

In terminal B:

```bash
source install/setup.bash

# 5) rooted bridge service visibility check
ros2 service list | grep -E '^/(navigate_to_pose|navigate_to_named_location|get_navigation_state|cancel_navigation)$' || true

# 6) direct client validator
python3 scripts/validate_mission_client_ros.py; echo EXIT_CODE:$?

# 7) facade validator
python3 scripts/validate_mission_agent_facade_ros.py; echo EXIT_CODE:$?
```

### Expected success outputs

`scripts/validate_mission_client_ros.py`:
- navigate step returns non-empty `goal_id`
- query step returns non-failure status (typically `status: "success"` and `nav_status: "in_progress"` in immediate checks)
- `EXIT_CODE:0`

`scripts/validate_mission_agent_facade_ros.py`:
- navigate step returns non-empty `goal_id`
- query step returns non-failure status (typically `status: "success"` and `nav_status: "in_progress"` in immediate checks)
- `EXIT_CODE:0`

### Pass/fail criteria

- **PASS:** both validators return `EXIT_CODE:0`, and both include navigate with non-empty `goal_id` plus non-failure query output.
- **FAIL:** either validator returns nonzero, or query reports failure semantics (for example `status: "failure"`).

### Stale-runtime safeguard (required)

After any Python bridge logic change, do **rebuild + fresh relaunch** before trusting validation results. Do not trust long-running nodes that started before code changes.

### Known caveat (documented, not solved here)

`gzserver` / simulation instability may appear during integrated launch. This can coexist with a passing mission-layer golden-path validation; treat simulation stability as a separate issue from the bridge/client/facade baseline contract.
