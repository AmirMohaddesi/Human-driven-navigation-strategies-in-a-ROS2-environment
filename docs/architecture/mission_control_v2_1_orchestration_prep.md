# Mission Control V2.1 — Multi-robot orchestration prep (design note)

## Purpose

Define the **smallest** increment beyond today’s validated **single-robot control plane** (§K, [mission_control_cli_policy.md](mission_control_cli_policy.md)) and **V2.0.1 cancel ownership** ([mission_control_v2_0_1_registry_ownership.md](mission_control_v2_0_1_registry_ownership.md)) so that **multi-robot coordination** does not rely on **ad hoc scripts**, **duplicate goal registries**, or **goal_id-only** APIs.

This note is **planning only**—no implementation obligation until approved. It does **not** redesign launch/runtime, broaden LangGraph, or add new mission contract types.

---

## Current baseline carried forward

**§K ([mission_system_runbook.md](mission_system_runbook.md))** — Navigate, query-state, and cancel for the **navigation control plane** slice; live-validated on **MissionClient**, **MissionAgentFacade.with_ros()**, and **CLI**. §K still frames **multi-robot coordination** as out of scope for that section; V2.1 **adds** an orchestration-prep layer **without** replacing §K.

**V2.0.1 (implemented)** — Bridge **cancel** classifies **owned_active** / **wrong_robot** / **unknown_goal**; **`goal_id → owning robot_id`** supports wrong-robot detection. **Wrong-robot cancel** is validated on client, facade, and CLI. Bridge remains the **source of truth** for ownership on cancel.

**CLI policy** — JSON primary; exit secondary; documented in [mission_control_cli_policy.md](mission_control_cli_policy.md).

**Layer B coordinator (already in repo)** — `multi_robot_mission_stack.coordinator` implements **team** named-navigation via **`assign_named_navigation`** (facade → navigate → wait for terminal), **`assign_named_sequence`**, and **`assign_named_parallel`** (rejects duplicate `robot_id` in one parallel batch). Each leg constructs its **own** `MissionAgentFacade.with_ros()`; coordination is **composition of per-robot legs**, not a second bridge. See [showcase_current_system.md](showcase_current_system.md): Layer B **consumes** Layer A; showcase explicitly notes **no cancellation orchestration** in the coordinator contract today.

**Facade entrypoint** — `MissionAgentFacade.handle_command` → `CommandAdapter` → `MissionGraph` → `MissionTools` → `MissionClient` / bridge services. Orchestration prep should **prefer this path** (or `MissionClient` directly) so behavior stays aligned with validated JSON contracts.

**Runtime assumptions** — `robot1` / `robot2` ↔ `robot1_ns` / `robot2_ns` in bridge; integrated swarm–class launch remains the canonical heavy validation context unless a slimmer profile is explicitly chosen later.

---

## Smallest orchestration problem to solve first

**Supervisor-safe interaction with goals when more than one robot exists:** an orchestrator (or coordinator helper) must **always** pass **`(robot_id, goal_id)`** together through the **existing** cancel/query/navigate surfaces and **rely on the bridge** for ownership classification—**not** infer ownership from Nav2 or keep a parallel “global goal table.”

The **first concrete gap** relative to the existing coordinator: Layer B can **assign** multi-robot legs but does not define a **small, ownership-aware cancel/observe** path for supervisors (e.g. “abort robot A’s goal” without accidentally calling cancel for robot B with A’s `goal_id` and misreading the outcome). V2.0.1 **`wrong_robot`** JSON exists; V2.1 **wires orchestration story to that contract** in one narrow vertical slice.

---

## Responsibilities of the orchestration layer (V2.1 scope)

- **Compose** calls to **MissionAgentFacade** and/or **MissionClient** using explicit **`robot_id`** per leg and per cancel/query.
- **Record** `(robot_id, goal_id)` **only as returned by the bridge/facade** for that leg—**operational cache** for the supervisor for the duration of a mission/step, not a second registry of truth.
- **Interpret** bridge responses using **§K + V2.0.1 + CLI policy** (e.g. distinguish **wrong_robot** vs **not_found** via JSON).
- **Expose** (in the first milestone) a **minimal** coordinator-level or script-level **recipe** for: navigate one robot → demonstrate **wrong-robot cancel** from a supervisor call → **correct-owner cancel**—all through facade/client, **no** new ROS services.
- **Validate** that path live (bounded: cleanup → launch → script → teardown), mirroring existing validator discipline.

---

## Non-responsibilities of the orchestration layer (V2.1)

- **Do not** store a **shadow copy** of `_active_goals` or re-derive ownership outside the bridge for cancel semantics.
- **Do not** change **bridge** cancel/query/navigate semantics unless a design review proves a **blocker** (default: **no bridge work** in V2.1 first milestone).
- **Do not** redesign **mission_contract v1**, add new **mission types**, or expand **LangGraph** scope “because we can.”
- **Do not** replace **assign_named_sequence** / **assign_named_parallel** with a new stack; **extend or document** patterns on top.
- **Do not** own **fleet scheduling**, cloud ops, or arbitrary N-robot policies beyond what the repo’s two-robot configuration already supports.

---

## Interaction with bridge / ownership contract

- **Cancel** and **query** must continue to go through **existing services**; orchestration passes **`robot_id`** that matches the robot that **owns** the `goal_id` for successful cancel, or **intentionally** receives **wrong_robot** / **not_found** when probing.
- **V2.0.1 wrong_robot** is the **control-plane signal** that a supervisor used the **wrong pair**; orchestration code should **never** remap that to “unknown” locally.
- **Terminal / query edge cases** (untracked `goal_id`, terminal cancel) remain **out of V2.1 first milestone** unless a blocker is proven—align with deferred items in the V2.0.1 note and §K.

---

## First milestone definition

**Milestone V2.1.0 — Ownership-aware supervisor cancel path (coordinator-scoped)**

1. Add a **small, documented API** in the **coordinator package** (or a single thin module used by it)—e.g. one function that issues **facade `cancel`** with explicit `(robot_id, goal_id)` and returns the **raw result dict** without reinterpretation—optional wrapper around `handle_command` only.
2. Add **one live ROS validator script** under `scripts/` that:
   - Uses **coordinator API** (or facade via coordinator helper—pick one for consistency) to: navigate **robot1** → cancel as **robot2** → assert **wrong_robot** contract → cancel as **robot1** → assert **success-shaped** cancel (§K-style `cancelling` / `not_cancellable`).
3. **No** changes to bridge, MissionClient protocol, or `.srv` schemas for this milestone unless a blocker appears.

This produces **visible multi-robot progress** (explicit cross-robot mistake + recovery) **without** a full fleet system or new mission types.

### Live validation (ownership-safe cancel)

**Implemented:** `scripts/validate_coordinator_ownership_cancel_ros.py` — live ROS against the integrated stack. Sequence: navigate **robot1** (named location **base**) → cancel **robot2** with **robot1**’s `goal_id` → **`wrong_robot`** contract → cancel **robot1** with the same `goal_id` → **success-shaped** cancel (`nav_status` **`cancelling`** or **`not_cancellable`**). Cancel steps use **`cancel_navigation_with_ros`** (Layer B; internally **`cancel_navigation_via_facade`**); navigate and optional **query-state** use one **`MissionAgentFacade.with_ros()`** session for submit + observe. Optional trailing **query-state** is log-only. **CLI parity:** `scripts/validate_mission_cli_ownership_cancel_ros.py` (and **`.sh`**) exercises the same sequence via **`mission-agent --ros`**.

---

## Validation plan

- **Live ROS** (same discipline as v1.1 / V2.0.1): `cleanup_baseline_runtime.sh pre` → build if needed → `fully_integrated_swarm` (or agreed canonical launch) → new script → `post` → `verify`.
- **Regression**: run existing **wrong_robot** client/facade/CLI validators and a **minimal** coordinator execute smoke if already part of CI/local habit—**no** broad script rewrites.
- **Offline**: optional unit test for pure **Python** helper that only forwards dicts (if added).

---

## Risks / non-goals

- **Risk — stale goals:** Bridge registry lifecycle is not fully TTL’d; supervisors should treat **goal_id** as **session-scoped** and not reuse across arbitrary time without re-navigation.
- **Risk — one Nav2 handle per namespace:** Query semantics for mismatched `goal_id` remain as today; V2.1 milestone does **not** fix query matrix.
- **Non-goal:** Full **cancellation orchestration** for all Layer B specs (retries, partial team abort)—only the **first** ownership-aware vertical slice.
- **Non-goal:** **LangGraph** as orchestration driver for V2.1.0.
- **Non-goal:** **Launch/runtime** redesign.

---

## Related documents

- [mission_system_runbook.md](mission_system_runbook.md) §K  
- [mission_control_v2_0_1_registry_ownership.md](mission_control_v2_0_1_registry_ownership.md)  
- [mission_control_cli_policy.md](mission_control_cli_policy.md)  
- [showcase_current_system.md](showcase_current_system.md) (Layer A/B)  
- [mission_contract_v1.md](../mission_contract_v1.md) (unchanged by this note)

---

## Revision

Draft for **V2.1 planning**; amend after manager approval before implementation.
