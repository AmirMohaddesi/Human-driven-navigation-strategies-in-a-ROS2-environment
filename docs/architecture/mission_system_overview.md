# Mission system architecture overview

Concise map of the ROS 2 + LangGraph mission stack as implemented in this repository. Paths are relative to the repo root.

---

## 1. High-level component diagram (ASCII)

```
+-----------------------------------------------------------------------------+
|                         future top-level agent facade                        |
|                    (orchestrator / LLM / API — not in repo yet)              |
+-----------------------------------------------------------------------------+
        |
        v
+-----------------------------------------------------------------------------+
|  src/multi_robot_mission_stack/agent/                                        |
|  CommandAdapter  ->  MissionGraph  ->  MissionTools  ->  MissionClient*      |
|       ^                 |                                                    |
|       |                 +--> MissionPolicy (allowlists)                      |
|  command_models.py      graph_state.py, mission_graph.py                   |
+-----------------------------------------------------------------------------+
        |
        |  MissionClient* implements MissionClientProtocol
        v
+------------------+     +------------------+     +---------------------------+
| MockMissionClient|     | (planned) ROS     |     | multi_robot_mission_stack_ |
| mock_mission_    | OR  | MissionClient     | --> | interfaces (.srv types)    |
| client.py        |     | (not in tree yet) |     +---------------------------+
+------------------+                           |
                                               v
+-----------------------------------------------------------------------------+
|  ROS 2 execution + bridge                                                    |
|  bridge/mission_bridge_node.py  --srv-->  bridge/nav2_client.py              |
|       (Nav2 NavigateToPose action per robot namespace)                       |
|  + Nav2, Gazebo/sim, other nodes under src/multi_robot_mission_stack/        |
+-----------------------------------------------------------------------------+
```

`*` Concrete ROS service client module is the intended plug-in; only the protocol and mock exist today.

---

## 2. Request flow diagram (ASCII)

Typical path from an external command to motion (when ROS is connected):

```
external dict          CommandAdapter.adapt()
  type/target/...  -->   internal graph request
                              |
                              v
                         MissionGraph.invoke()
                              |
                    +---------+---------+
                    | parse_request     |
                    +---------+---------+
                              |
                              v
                    +---------+---------+
                    | policy_check      |  MissionPolicy.evaluate()
                    +---------+---------+
                              |
                    +---------+---------+
                    | route + execute   |  MissionTools.*()
                    +---------+---------+
                              |
                              v
                    MissionClientProtocol
                    (mock now / ROS client later)
                              |
                              v
                    (optional) mission_bridge services
                              |
                              v
                         Nav2 / robots
```

Adapter failures and policy denials return structured `status: failed` dicts without calling tools.

---

## 3. Layers (short)

### ROS 2 execution layer

- **Where:** `src/multi_robot_mission_stack/` (nodes, `launch/`, `config/`, worlds, maps, etc.) and third-party stacks (Nav2, Gazebo, TurtleBot3, …).
- **Role:** Run robots, localization, navigation actions, simulation, sensors. No LangGraph dependency here.

### Mission bridge

- **Where:** `bridge/mission_bridge_node.py`, `bridge/nav2_client.py`.
- **Role:** `rclpy` node exposing mission-level **services** (`navigate_to_pose`, `navigate_to_named_location`, `get_navigation_state`, `cancel_navigation`) implemented over per-robot Nav2 **NavigateToPose** actions. Service types live in `src/multi_robot_mission_stack_interfaces/srv/`.
- **Launch:** `mission_bridge_node` is included from `launch/fully_integrated_swarm.launch.py` (not every launch file).

### MissionClient / protocol boundary

- **Where:** `agent/mission_client_protocol.py` defines `MissionClientProtocol`. `agent/mock_mission_client.py` implements it in-process for tests/dev.
- **Role:** Single abstraction LangGraph-facing code should use for mission I/O. A future ROS-backed client would call the bridge services and map responses to plain `dict`s—**that concrete client is not present in the repo yet**; the protocol documents the contract.

### MissionTools

- **Where:** `agent/mission_tools.py`.
- **Role:** Validates tool inputs (robot id, coordinates, goal id, location name), calls the protocol, normalizes outputs to stable dicts. No ROS imports.

### MissionPolicy

- **Where:** `agent/mission_policy.py`, `agent/policy_config.py` (`MissionPolicyConfig`, `build_default_policy_config()`).
- **Role:** Allowlists (`allowed_actions`, `allowed_robot_ids`, `allowed_locations`) evaluated **before** tool execution inside `MissionGraph`. Not LangGraph-specific logic beyond orchestration.

### CommandAdapter

- **Where:** `agent/command_adapter.py`, `agent/command_models.py`.
- **Role:** Maps **external** command shapes (`type` / `target` / …) to the **internal** graph request shape (`action` + fields). Shape validation only; no policy, no ROS.

### MissionGraph

- **Where:** `agent/mission_graph.py`, `agent/graph_state.py`.
- **Role:** Small LangGraph `StateGraph`: parse → policy → route → execute via `MissionTools`. Deterministic; no LLM. Depends on `langgraph` (see `setup.py`).

### Future top-level agent facade

- **Where:** not implemented.
- **Role:** Intended place for LLM, API gateway, or multi-step reasoning that calls `CommandAdapter` + `MissionGraph` (or internal requests directly) without importing `rclpy`.

---

## 4. Why this separation exists

| Boundary | Reason |
|----------|--------|
| Protocol vs bridge | Agent code stays import-clean and testable; ROS types and service names stay behind one implementation. |
| MissionTools vs MissionGraph | Reusable validation/normalization whether the caller is a graph, a script, or a future facade. |
| MissionPolicy vs adapter | Policy encodes **who/what is allowed**; adapter encodes **wire format**. Changing operators’ JSON should not rewrite allowlists. |
| CommandAdapter vs graph | External API can evolve without changing graph nodes; graph keeps a single internal request schema. |
| Mock client | CI and dev machines without Ubuntu/ROS can run policy, adapter, graph, and tools. |

---

## 5. What can be tested without ROS

- `CommandAdapter` / `command_models` (adapt success and failure dicts).
- `MissionPolicy` / `policy_config` (`evaluate` allow/deny).
- `MissionGraph.invoke()` with `MissionTools(MockMissionClient())`.
- `MissionTools` with `MockMissionClient` (all four protocol methods).
- Pure-Python imports of `agent/*` modules (no `rclpy` in those files).

---

## 6. What still requires Ubuntu / ROS validation

- Building and running `multi_robot_mission_stack_interfaces` (ament_cmake + `rosidl` codegen).
- `mission_bridge_node` + `nav2_client` against live Nav2 action servers and correct namespaces.
- End-to-end service names, remaps, and timing (async goal acceptance vs `MissionTools` expectations).
- The **planned** ROS `MissionClient` implementation: service clients, `rclpy` init lifecycle, and parity with `MockMissionClient` response shapes.
- Full stack launches (`launch/*.py`) with simulation or hardware.

---

## Package map (reference)

| Package / area | Path |
|----------------|------|
| Application + bridge + agent | `src/multi_robot_mission_stack/` |
| Generated mission .srv | `src/multi_robot_mission_stack_interfaces/` |
| This doc | `docs/architecture/mission_system_overview.md` |
