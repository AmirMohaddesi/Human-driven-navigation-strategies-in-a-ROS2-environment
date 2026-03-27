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
ros2 service list | grep mission_bridge_node
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

- `/mission_bridge_node/navigate_to_pose`
- `/mission_bridge_node/navigate_to_named_location`
- `/mission_bridge_node/get_navigation_state`
- `/mission_bridge_node/cancel_navigation`

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
