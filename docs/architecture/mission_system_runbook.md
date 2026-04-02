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

**CLI JSON vs shell exit:** see [mission_control_cli_policy.md](mission_control_cli_policy.md).

---

## E. Direct validation scripts

Run from the **repository root** (scripts add `src/` to `sys.path`; prefer the same shell where `install/setup.bash` is sourced so generated interfaces resolve).

```bash
# Optional first: per-robot Nav2 startup (lifecycles, namespaced /map, /tf, amcl tf_broadcast)
bash scripts/check_robot_readiness.sh robot1_ns; echo READINESS_EXIT:$?

python3 scripts/validate_mission_client_ros.py
python3 scripts/validate_mission_agent_facade_ros.py
python3 scripts/validate_mission_agent_facade_cancel_ros.py
bash scripts/validate_mission_cli_cancel_ros.sh
python3 scripts/validate_mission_cancel_invalid_goal_ros.py
python3 scripts/validate_mission_cancel_terminal_goal_ros.py
python3 scripts/validate_mission_agent_facade_cancel_invalid_goal_ros.py
bash scripts/validate_mission_cli_cancel_invalid_goal_ros.sh
python3 scripts/validate_coordinator_ownership_cancel_ros.py
bash scripts/validate_mission_cli_ownership_cancel_ros.sh
python3 scripts/validate_mission_client_ownership_cancel_ros.py
```

| Script | Validates |
|--------|-----------|
| `check_robot_readiness.sh` | **Operator readiness** (not mission validation): lifecycles `map_server` / `amcl` / `bt_navigator`, **`/{ns}/map`**, **`/{ns}/tf`** publishers, **`amcl` `tf_broadcast`**, **`slam_toolbox`** on `/tf`. Exits `0` ready, `1` not ready, `2` partial, `3` no `ros2`. |
| `validate_mission_client_ros.py` | `MissionClient`: `navigate_to_named_location` → `get_navigation_state` |
| `validate_mission_agent_facade_ros.py` | `MissionAgentFacade.with_ros()` with external navigate + query commands |
| `validate_mission_agent_facade_cancel_ros.py` | `MissionAgentFacade.with_ros()`: navigate → cancel → query-after-cancel |
| `validate_mission_cancel_ros.py` | `MissionClient`: same cancel flow (direct bridge client) |
| `validate_mission_cli_cancel_ros.sh` / `.py` | **`mission-agent --ros`**: navigate → cancel → query-state (user-facing CLI) |
| `validate_mission_cancel_invalid_goal_ros.py` | `MissionClient`: cancel with unknown `goal_id` (invalid-goal contract, **§K**) |
| `validate_mission_cancel_terminal_goal_ros.py` | `MissionClient`: **V2.0.2** terminal-goal cancel — navigate → wait until terminal → cancel → `success` / `not_cancellable` / `Goal is already complete` ([mission_control_v2_0_2_terminal_query_semantics.md](mission_control_v2_0_2_terminal_query_semantics.md)) |
| `validate_mission_agent_facade_cancel_invalid_goal_ros.py` | Facade ROS path: same invalid-goal cancel contract (**§K**) |
| `validate_mission_cli_cancel_invalid_goal_ros.sh` / `.py` | CLI: same invalid-goal cancel contract (**§K**) |
| `validate_coordinator_ownership_cancel_ros.py` | V2.1 **ownership-safe cancel** (live): navigate **robot1** → cancel **robot2** with robot1 `goal_id` → **wrong_robot** → cancel **robot1** → success cancel contract; see [mission_control_v2_1_orchestration_prep.md](mission_control_v2_1_orchestration_prep.md) |
| `validate_mission_cli_ownership_cancel_ros.sh` / `.py` | CLI: same V2.1 **ownership-safe cancel** flow as coordinator validator (**§K** / V2.0.1 contracts) |
| `validate_mission_client_ownership_cancel_ros.py` | `MissionClient`: same V2.1 **ownership-safe cancel** flow (direct bridge client) |

Nonzero exit on failure; compact JSON lines on stdout. **§K** freezes the navigation-control-plane JSON semantics these scripts exercise (happy path + invalid-goal cancel).

**Readiness scope:** `check_robot_readiness.sh` encodes the **§J** startup contract in one command. The Python validators are **mission-layer / bridge smoke tests** (goal acceptance + query semantics) and do **not** replace that probe. For rationale and manual fallbacks, see **§J** below.

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
| `nav_status` / bridge status `rejected` (navigate) | Nav2 / bridge | Namespaces (`robot1_ns`), goal feasible. Follow **§J** (lifecycle, **`/{robot}_ns/map`**, **`/{robot}_ns/tf`** / `map`→`odom`). **Do not** treat root **`/merged_map`** or a generic root **`/map`** as the per-robot readiness gate. |
| `status`: `failed` with adapter wording (e.g. unknown target) | CommandAdapter | External command shape: `type` / `target` / required fields. |

---

## H. Current supported commands

Through the **facade and graph**, the supported **external** operations are:

1. **Navigate to named location** — `mission-agent … navigate --robot-id … --location-name …`
2. **Query navigation state** — `mission-agent … query-state --robot-id … --goal-id …`
3. **Cancel navigation** — `mission-agent … cancel --robot-id … --goal-id …`

**Live check (facade cancel path):** `python3 scripts/validate_mission_agent_facade_cancel_ros.py` (after the same ROS prerequisites as other validators). Additional shapes (e.g. navigate-to-pose) exist via structured commands and bridge services (§F); they are not all listed here.

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
bash scripts/cleanup_baseline_runtime.sh pre

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

# 5b) if startup health is in doubt: per-robot readiness (see §J); then continue only if acceptable
bash scripts/check_robot_readiness.sh robot1_ns; echo READINESS_EXIT:$?

# 6) direct client validator
python3 scripts/validate_mission_client_ros.py; echo EXIT_CODE:$?

# 7) facade validator
python3 scripts/validate_mission_agent_facade_ros.py; echo EXIT_CODE:$?

# 8) mandatory post-run teardown cleanup
bash scripts/cleanup_baseline_runtime.sh post

# 9) optional explicit verification-only check
bash scripts/cleanup_baseline_runtime.sh verify
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

- **PASS:** both validators return `EXIT_CODE:0`, and both include navigate with non-empty `goal_id` plus query `status` not failed/failure and `nav_status` not `unknown`/`not_found`.
- **FAIL:** either validator returns nonzero, query reports failure semantics, or query returns unresolved state markers like `nav_status: "unknown"` / `No active goal`.

If failures look like **startup** (nothing to navigate, TF/map warnings), use **§J** readiness probes—not merged map or undocumented root `/map` expectations.

### Stale-runtime safeguard (required)

After any Python bridge logic change, do **rebuild + fresh relaunch** before trusting validation results. Do not trust long-running nodes that started before code changes.

### Known caveat (documented, not solved here)

`gzserver` / simulation instability may appear during integrated launch. This can coexist with a passing mission-layer golden-path validation; treat simulation stability as a separate issue from the bridge/client/facade baseline contract.

---

## J. Per-robot navigation readiness and TF ownership (`fully_integrated_swarm`)

**Operator summary:** per-robot navigation is **not** gated on **`/merged_map`** or a global merged **`/map`**. It depends on **namespaced** `map_server` + AMCL + Nav2 + **SLAM Toolbox** supplying **`map` → `odom`** (AMCL has **`tf_broadcast: false`** here). **`/merged_map`** is **auxiliary** (visualization / tooling).

This section closes the **runtime contract** for the integrated TurtleBot3 + Nav2 + SLAM + bridge stack: what each robot’s navigation actually depends on, what merged mapping is for, who supplies **`map` → `odom`**, and what to treat as **ready** for validation or debugging.

### Per-robot navigation dependency (fact)

- Each robot uses `localization_navigation_launch.py` under **`{robot}_ns`** (e.g. `robot1_ns`): **`map_server`** (static map from `maps/map.yaml` passed as the launch `map` argument), **`amcl`**, and the full Nav2 navigation lifecycle set (`bt_navigator`, planners, controllers, etc.).
- Topics are namespaced (e.g. **`/robot1_ns/map`** from `map_server`, **`/robot1_ns/odom`**, **`/robot1_ns/scan`**). Nav2 costmaps and `bt_navigator` use **`global_frame: map`** per `config/nav2_multirobot_params_all_copy.yaml`.
- **`merge_map_node`** and **`/merged_map`** are **not** wired into this Nav2 bringup; per-robot navigation does **not** depend on a global merged occupancy grid for the golden path.

### Merged map role (fact)

- `merge_map_node` subscribes to each robot’s **`local_map`** (SLAM output), publishes **`/merged_map`**, and publishes a global static **`world` → `map`** transform for visualization. Per-robot **`updated_map` republish is commented out** in `merge_map_node.py`.
- Treat merged output as **auxiliary** (e.g. multi-robot RViz / downstream tooling), **not** on the critical path for namespaced Nav2 navigation.

### `map` → `odom` ownership (fact + inference)

**Observed at runtime** (with `fully_integrated_swarm` up and `/robot1_ns/tf` present):

```bash
source install/setup.bash
ros2 topic info /robot1_ns/tf -v
```

Publishers on **`/robot1_ns/tf`** include **`amcl`**, **`slam_toolbox`** (two publisher endpoints on the same node), **`robot_state_publisher`**, and **`turtlebot3_diff_drive`** (Gazebo diff drive). So several nodes contribute to the namespaced TF graph; **`map` → `odom`** is not visible from `topic info` alone.

**Additional check** (recommended when closing ownership):

```bash
ros2 param get /robot1_ns/amcl tf_broadcast
```

In this repository, **`amcl.tf_broadcast` is `false`** in `nav2_multirobot_params_all_copy.yaml` and matches runtime (`Boolean value is: False`). Per Nav2 AMCL documentation, **`tf_broadcast: false` prevents AMCL from publishing the transform between the global frame and the odometry frame** (`map` ↔ `odom`). AMCL may still appear as a **`/tf` publisher** in `ros2 topic info`; that does not contradict the parameter (see Nav2 “Configuring AMCL”, `tf_broadcast`).

**Inference (stack design):** With **`tf_broadcast: false`** on AMCL and **`slam_toolbox`** running in **`mode: localization`** with **`map_frame: map`** and **`odom_frame: odom`** (`config/slam_online_async.yaml`), **SLAM Toolbox is the intended supplier of the per-robot `map` → `odom` transform** for this launch. **`initial_pose_publisher`** uses a namespaced TF lookup **`map` → `odom`** to seed AMCL’s **`initialpose`**; that lookup succeeds once that edge exists in the buffer (from SLAM, not from `merge_map_node`, which publishes on the **global** TF tree).

If you see conflicting or unstable `map`/`odom` behavior, confirm the above publishers and parameters before changing delays or mission code.

### Minimum readiness signals

**Fact (what the repo encodes):**

- **`lifecycle_manager_localization`** has activated **`map_server`** and **`amcl`**; **`lifecycle_manager_navigation`** has activated navigation nodes including **`bt_navigator`** (see `localization_navigation_launch.py`).
- Messages on **`/{robot}_ns/map`** from **`map_server`** (static map for Nav2 costmaps / AMCL map topic).
- A coherent namespaced TF chain usable by Nav2: **`map` → `odom` → `base_footprint` / `base_link`** (plus robot model and diff-drive edges).

**Inference (operational):**

- For **`map` → `odom`**, rely on **SLAM Toolbox** given **`amcl.tf_broadcast: false`**; do **not** assume AMCL publishes that edge.
- Golden-path validators (`scripts/validate_mission_client_ros.py`, `scripts/validate_mission_agent_facade_ros.py`) assert **bridge-level** readiness (non-empty **`goal_id`**, non-degenerate **`nav_status`**) and do **not** replace the TF/map checks above for deep navigation debugging.

### Troubleshooting: startup readiness (check in this order)

**Preferred one-shot probe:** `bash scripts/check_robot_readiness.sh [namespace]` (default `robot1_ns`). Same shell must have **`source install/setup.bash`**. Exit codes: **`0`** ready, **`1`** not ready, **`2`** partially ready, **`3`** `ros2` missing.

Use **`robot1_ns`** (or the failing robot’s namespace) and the same shell where **`source install/setup.bash`** was run.

1. **Lifecycle (localization then navigation)**  
   `ros2 lifecycle get /robot1_ns/map_server` and `.../amcl` → expect **`active [3]`**.  
   `ros2 lifecycle get /robot1_ns/bt_navigator` (or `lifecycle_manager_navigation` via `ros2 node list`) → navigation side **active**.

2. **Namespaced static map (Nav2 / AMCL)**  
   Confirm **`/robot1_ns/map`** is publishing (e.g. `ros2 topic echo /robot1_ns/map --field header.stamp --once` with a few seconds timeout). This is the **`map_server`** topic for this stack—not **`/merged_map`**, not root **`/map`**.

3. **SLAM + TF (`map` → `odom`)**  
   `ros2 topic info /robot1_ns/tf -v` → expect publishers including **`slam_toolbox`** and **`amcl`** (among others).  
   `ros2 param get /robot1_ns/amcl tf_broadcast` → should be **`False`**; per Nav2, AMCL does **not** publish **global↔odom** here—**SLAM Toolbox** is the intended **`map` → `odom`** source.  
   Optionally sample TF: `ros2 topic echo /robot1_ns/tf` until you see a transform with **`frame_id: map`** and **`child_frame_id: odom`**.

4. **What not to use as a gate**  
   Absence or delay of **`/merged_map`** does **not** by itself explain per-robot Nav2 failure. Do not assume **`merge_map_node`** or a global **`/map`** is required for **`robot1_ns`** navigation in this launch.

5. **Then** re-run **`scripts/validate_mission_client_ros.py`** / facade validator if the bridge path is still the question.

### Fact vs inference (summary)

| Topic | Fact | Inference |
|-------|------|-----------|
| Nav2 uses per-robot `map_server` + AMCL + nav stack | Yes | — |
| Merged `/merged_map` required for per-robot Nav2 | No | — |
| Several nodes publish on `/{ns}/tf` | Yes (from `ros2 topic info -v`) | — |
| `amcl.tf_broadcast` is false at runtime | Yes (param file + `ros2 param get`) | — |
| Which publisher emits each transform pair | — | `topic info` does not label edges; **`map`→`odom` attributed to `slam_toolbox`** given AMCL tf_broadcast off + SLAM localization config |
| AMCL’s `/tf` publisher when `tf_broadcast` is false | — | Treat as Nav2 implementation detail; verify against Nav2 release notes if behavior surprises |

---

## K. Navigation control plane contract (Mission Control v1.1)

**Authoritative for:** live ROS navigation commands against the mission bridge—**named-location navigate**, **query navigation state**, **cancel navigation**—for **one robot at a time** in the configuration the validators use (e.g. `robot1` / `robot1_ns`). This is the **current mission slice** only: it does **not** define multi-robot coordination, new mission types, or LangGraph behavior beyond what reaches these bridge services.

### Validated surfaces

The following surfaces are **live-validated** against the integrated stack (see **§E** for script names):

| Surface | Mechanism |
|---------|-----------|
| **MissionClient** | Direct ROS service calls to the bridge (`navigate_to_named_location`, `get_navigation_state`, `cancel_navigation`). |
| **MissionAgentFacade** | `MissionAgentFacade.with_ros()` handling structured navigate / query / cancel commands (same bridge underneath). |
| **`mission-agent` CLI** | `mission-agent --ros …` or `python -m multi_robot_mission_stack.agent.cli --ros …` (real operator entrypoint). |

### Happy-path contract (validated)

Operators and integrators may assume the following **when the stack is healthy** (readiness: **§J**, cleanup: **§I**):

1. **Navigate → query**  
   After a successful navigate to a named location, the returned `goal_id` can be used with query-state; validators expect a **non-failure** outcome with a **resolved** navigation state (not `unknown` / `not_found` for that active goal in the immediate check).

2. **Navigate → cancel → query-after-cancel**  
   **Success at a high level** (as encoded in **§E** cancel scripts): navigate yields a non-empty `goal_id` with non-failure navigate semantics; cancel returns non-failure `status` with `nav_status` in **`cancelling`** or **`not_cancellable`**; the immediate follow-up query returns `status: success` with a **non-empty** `nav_status` that is **not** `unknown` or `not_found`.

### Invalid-goal cancel contract (validated)

For **cancel** with a `goal_id` that the bridge has **no record of** for that robot (validators use a bogus id such as `bogus-goal-id-for-validator`), the JSON response **must** match:

| Field | Value |
|-------|--------|
| `status` | `"failure"` |
| `nav_status` | `"not_found"` |
| `message` | `Goal id not found for this robot` |

This contract is asserted on **MissionClient**, **facade**, and **CLI** paths in **§E**.

### Semantic notes (not bugs for v1.1)

- **Cancel “success”** (request accepted) may appear as `nav_status` **`cancelling`** or **`not_cancellable`** depending on Nav2 / timing; both are treated as successful cancel initiation in the cancel validators.
- **Query-after-cancel** may still reflect an **in-flight** or **terminal** Nav2-derived state briefly; integrators should not assume an immediate “blank” world state.
- **CLI process exit code:** authoritative policy is [mission_control_cli_policy.md](mission_control_cli_policy.md). **JSON** remains authoritative for semantics (e.g. **wrong_robot** vs **not_found**); exit code is a **secondary** automation signal per that policy.
- **Wrong-robot cancel** is live-validated (**§E** ownership scripts, V2.0.1). **Terminal-goal cancel (V2.0.2, MissionClient only):** after Nav2 reports a terminal `nav_status` for the goal, `cancel_navigation` returns `status: success`, `nav_status: not_cancellable`, `message: Goal is already complete` — see `validate_mission_cancel_terminal_goal_ros.py` and [mission_control_v2_0_2_terminal_query_semantics.md](mission_control_v2_0_2_terminal_query_semantics.md). **Query semantics** for untracked / wrong-robot / terminal edge cases remain **not** frozen here unless separately validated.

### Explicitly out of scope (this section does not freeze)

- Multi-robot orchestration and cross-robot goal ownership.
- Broader mission types, mission graphs, or policies beyond this navigate/query/cancel slice.
- Registry or active-goal store redesign.
- CLI exit semantics are governed by [mission_control_cli_policy.md](mission_control_cli_policy.md) (not ad hoc runbook edits).
- Wrong-robot **query** and broader query-matrix edge cases until covered by dedicated validation (terminal-goal **cancel** on MissionClient is covered by **§E** / V2.0.2 note above).
- LangGraph scope expansion or launch/runtime redesign.
