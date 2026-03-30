# Current System Showcase

## 1. What the System Can Do Now

- Run **named-navigation** missions per robot through the ROS2 **mission bridge** and **agent facade** (Layer A).
- Coordinate **team** missions in **sequence** or **parallel** over those primitives, with optional batching of multiple specs (Layer B).
- **Inspect** mission specs offline (preflight + preview), including **checked** variants that attach invariant validation payloads.
- **Execute** single- and batch-spec missions against a live stack, including **checked** variants that validate summary shapes after run.
- Rely on a **frozen mission contract v1** (spec shape, execution/inspection/summary tags, lightweight validators) and a static **API manifest** with resolver, request validation, and **plan** helper for call planning.
- Use a **thin planner seam** (Layer C) to turn structured **intent** into a v1 spec, validate contract, then run **inspect_checked** before **execute_checked** (with demos that stub live execute when ROS is absent).

## 2. Layer A — Execution

What is **real** and **verified** (per repo tests, scripts, and handoff):

- **Mission bridge** (`mission_bridge_node`) and **MissionAgentFacade** as the execution boundary used by the coordinator’s navigation calls.
- **Per-leg execution**: submit named navigation, wait for terminal navigation state, surface outcomes in compact dicts (`assign_named_navigation` and helpers in the coordinator that call into the facade).
- **Supporting scripts** such as `validate_mission_agent_facade_ros.py`, `validate_mission_client_ros.py`, and in-process sequence/parallel/wait validators for the execution stack—not part of Layer B contract, but evidence of exercised runtime paths.

Layer A is **not** redesigned here; Layer B **consumes** it.

## 3. Layer B — Coordination and Contract

**Package:** `multi_robot_mission_stack.coordinator`

| Area | Capabilities |
|------|----------------|
| **Single spec** | `inspect_team_named_mission_spec`, `inspect_team_named_mission_spec_checked`, `execute_team_named_mission_spec`, `execute_team_named_mission_spec_checked`, plus `run_*`, `preflight_*`, `preview_*`, `summarize_team_named_mission_result` |
| **Batch specs** | Same pattern with `*_specs` / `*_specs_checked` and `summarize_team_named_mission_specs_result` |
| **Modes** | `sequence` and `parallel` team dispatch (`assign_named_sequence`, `assign_named_parallel`, `assign_team_named_mission`, normalization) |
| **Contract v1** | `validate_team_named_mission_spec_contract`, `validate_team_named_mission_specs_contract`, `validate_team_named_mission_execution_contract`, `validate_team_named_mission_specs_execution_contract`; stable payloads carry additive `"version": "v1"` where frozen |
| **Manifest / plan** | `get_team_named_mission_api_manifest`, `validate_team_named_mission_api_manifest`, `resolve_team_named_mission_api_entrypoint`, `validate_team_named_mission_api_request`, `plan_team_named_mission_api_call` |
| **Invariant validators** | `validate_team_named_mission_inspection`, `validate_team_named_mission_specs_inspection`, `validate_team_named_mission_summary`, `validate_team_named_mission_specs_summary` |

**Source of truth for contract:** `docs/mission_contract_v1.md`.

## 4. Layer C — Planner Seam

**Module:** `multi_robot_mission_stack.planner_seam`

- **`build_team_named_mission_spec_v1_from_intent`**: maps a small intent dict (`mode`, `steps`, optional `options`) to a **v1** mission spec and runs **contract** validation.
- **`run_planner_team_named_mission_checked`**: contract → **`inspect_team_named_mission_spec_checked`** → **`execute_team_named_mission_spec_checked`** only if inspection succeeds; injectable hooks for tests.

**Does not:** replan, retry, recover, or change Layer B semantics or payloads.

**Demos / notes:** `docs/planner_seam_demo.md`, `scripts/planner_seam_demo_report.py`.

## 5. Demos and Validation Scripts

**Handoff / manager**

- `docs/handoff/manager_handoff_summary.md` — checkpoint summary and quality bar.
- `docs/handoff/capability_snapshot.md` — capability table.
- `docs/handoff/golden_path_demo.md` — checked single/batch inspect & execute.
- `docs/handoff/non_goals_deferred.md` — explicit deferrals.

**Golden path driver**

- `scripts/demo_layer_b_golden_path.sh` — offline inspect-checked by default; `--with-ros` for execute-checked.

**Contract**

- `docs/mission_contract_v1.md`
- `scripts/validate_mission_contract_v1.py`

**Layer B (representative; more under `scripts/`)**

- Offline: `validate_coordinator_inspect_spec.py`, `validate_coordinator_inspect_checked.py`, `validate_coordinator_batch_inspect.py`, `validate_coordinator_batch_inspect_checked.py`, `validate_coordinator_api_manifest.py`, `validate_coordinator_api_request.py`, `validate_coordinator_api_resolver.py`, `validate_coordinator_api_plan.py`, `validate_coordinator_single_result_summary.py`, `validate_coordinator_batch_result_summary.py`, …
- Live ROS: `validate_coordinator_execute_spec_ros.py`, `validate_coordinator_execute_checked_ros.py`, `validate_coordinator_execute_batch_ros.py`, `validate_coordinator_execute_batch_checked_ros.py`, and coordinator/sequence/parallel ROS smoke scripts.

**Planner seam**

- `docs/planner_seam_demo.md`
- `scripts/planner_seam_demo_report.py`

**This showcase**

- `docs/showcase_current_system.md` (this file)
- `scripts/showcase_current_system.py` — static index of surfaces (no ROS, no mission execution).

## 6. Example End-to-End Flows

**Offline checked inspection**

1. Build or load a mission spec (dict with `mode` / `steps`; add `"version": "v1"` for strict contract demos).
2. Call `inspect_team_named_mission_spec_checked(spec)` (or batch equivalent).
3. Observe top-level `ok`, nested `inspection`, and `validation` payloads (and `version` on stable returns).

**Live checked execution**

1. ROS workspace + mission bridge / Nav2 as used in existing execute scripts.
2. Run `validate_coordinator_execute_checked_ros.py` or batch variant, or call `execute_team_named_mission_spec_checked` from Python.
3. Top-level result includes run summary plus checked validation of execution summary invariants.

**Planner seam flow**

1. Provide intent dict to `run_planner_team_named_mission_checked` (or build first with `build_team_named_mission_spec_v1_from_intent`).
2. Contract validation → inspect_checked → execute_checked only if inspect passes.
3. For a laptop/CI demo without ROS, use `planner_seam_demo_report.py` (stubbed execute) to see the same stages in a report.

## 7. What Is Explicitly Not Implemented Yet

Aligned with `docs/handoff/non_goals_deferred.md` and contract non-goals:

- No **LangGraph / full planner** graph bound above Layer B beyond the **minimal seam**.
- No **retries, recovery, or cancellation orchestration** in the coordinator contract.
- No **new mission modes** or batch policies unless explicitly scoped.
- No **ROS message/service redesign** for Layer B; no **coordinator redesign**.
- **Simulator-only debugging** and **distributed execution hardening** are separate workstreams.

## 8. Recommended Demo Order

1. `python3 scripts/showcase_current_system.py` — quick static map of layers and artifacts.
2. `python3 scripts/validate_mission_contract_v1.py` — contract validators + manifest.
3. `./scripts/demo_layer_b_golden_path.sh` — offline checked inspect (single + batch).
4. Read `docs/handoff/golden_path_demo.md` then run live execute steps if ROS is available.
5. `python3 scripts/planner_seam_demo_report.py` — Layer C report (stub execute).
6. Skim `docs/mission_contract_v1.md` and `docs/handoff/manager_handoff_summary.md` for sign-off context.
