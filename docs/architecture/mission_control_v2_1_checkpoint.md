# Mission Control V2.1 — Checkpoint (complete)

## Purpose

Document the **closed** V2.1 **orchestration-prep** slice: supervisor-facing **ownership-safe cancel** wiring in Layer B over the existing bridge and validated client/facade/CLI surfaces, **without** a second goal registry, bridge semantic changes, or mission-type expansion. This file is the **checkpoint record**; design rationale and history remain in [mission_control_v2_1_orchestration_prep.md](mission_control_v2_1_orchestration_prep.md).

## What V2.1 proved

- **Layer B cancel forwarding:** `cancel_navigation_via_facade` issues a single **`MissionAgentFacade.handle_command`** cancel payload (`type` / `target` / `robot_id` / `goal_id`) and returns the **raw** result dict (bridge/facade remains the authority for **wrong_robot** vs owned cancel).
- **Layer B ROS entrypoint:** `cancel_navigation_with_ros` opens a facade session, delegates to `cancel_navigation_via_facade`, closes — no coordinator-side ownership remapping.
- **Live cross-robot cancel contract** on the coordinator path: navigate **robot1** → cancel as **robot2** with robot1’s `goal_id` → **`wrong_robot`** → cancel as **robot1** → success-shaped cancel (`nav_status` **`cancelling`** or **`not_cancellable`**) per §K / V2.0.1-style expectations.
- **Parity:** The same ownership-safe sequence is exercised on **`MissionClient`** and **CLI** (`mission-agent --ros`) via the existing validator scripts listed below.
- **Offline lock:** Unit tests mock the facade boundary and assert cancel command shape, **identity** pass-through of the return dict, and `.strip()` on ids — deterministic regression without ROS.

## Accepted artifacts and tests

| Kind | Location |
|------|----------|
| Implementation | `src/multi_robot_mission_stack/multi_robot_mission_stack/coordinator/coordinator.py` — `cancel_navigation_via_facade`, `cancel_navigation_with_ros`; exported from `coordinator/__init__.py` |
| Live ROS — coordinator | `scripts/validate_coordinator_ownership_cancel_ros.py` |
| Live ROS — MissionClient | `scripts/validate_mission_client_ownership_cancel_ros.py` |
| Live ROS — CLI | `scripts/validate_mission_cli_ownership_cancel_ros.py`, `scripts/validate_mission_cli_ownership_cancel_ros.sh` |
| Offline | `tests/test_coordinator.py` — `test_cancel_navigation_via_facade_calls_handle_command_with_cancel_navigation_shape`, `test_cancel_navigation_via_facade_strips_robot_id_and_goal_id` |
| Optional long-budget confidence | `scripts/validate_coordinator_parallel_ownership_smoke_ros.py` (two navigates submitted without terminal wait, then wrong_robot + owner cancel; requires **dual-namespace** readiness — treat readiness timeout as **environment not ready**, not validator failure; see prep doc **Fast deterministic checks vs heavy parallel smoke**) |

**Fast default command:** `pytest tests/test_coordinator.py -k cancel_navigation_via_facade`

## What remains explicitly out of scope for V2.1

- **Bridge** cancel/query/navigate **semantic** changes, **launch/runtime** redesign, **LangGraph** expansion, **new mission contract types**, **shadow** goal registries, and broad **Layer B sequence/parallel** redesign (orchestration still composes per-robot legs as before).
- **Terminal / query matrix** edge cases and unified **V2.0.2** semantics — **not** part of this slice; see **Next active work item** below.
- Using **parallel two-robot smoke** as a **default** CI gate (dual readiness is heavy; deterministic tests are the default regression).

## Why V2.1 stops here

Engineering and management agreed the **orchestration-prep** goal is met: explicit `(robot_id, goal_id)` cancel through validated surfaces, **wrong_robot** observable on live paths, and a **thin** Layer B API with offline tests. Further V2.1 work would **stretch** scope without changing that outcome; the slice is **frozen** as complete.

## Next active work item: V2.0.2 terminal/query semantics

**V2.0.2** (not started in this repo as of this checkpoint) is the **next** active control-plane focus: **terminal goal** behavior and **query-state** semantics where they remain ambiguous or deferred relative to §K and [mission_control_v2_0_1_registry_ownership.md](mission_control_v2_0_1_registry_ownership.md). V2.1 **does not** implement or specify V2.0.2; it only **hands off** priority to that track.
