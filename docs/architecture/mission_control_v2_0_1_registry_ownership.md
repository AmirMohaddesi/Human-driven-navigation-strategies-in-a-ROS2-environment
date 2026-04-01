# Mission Control V2.0.1 — Registry and ownership contract (design note)

## Purpose

Freeze **goal ownership** and **bridge registry semantics** so multi-robot orchestration (later) and upper layers (facade, CLI, LangGraph) never infer ownership from Nav2 side effects alone or duplicate goal state outside `mission_bridge_node`.

This note is the **design closure** for milestone **V2.0.1**. It does **not** change code by itself. Implementation must stay a **minimal evolution** of existing `MissionBridgeNode._active_goals` behavior and existing ROS services—**no rewrite**, no orchestration layer, no launch/runtime redesign.

---

## Current baseline carried forward

The following is **validated in v1.1** and remains true unless this note explicitly **adds** a contract (see [mission_system_runbook.md](mission_system_runbook.md) **§K**).

- **Surfaces:** `MissionClient`, `MissionAgentFacade.with_ros()`, `mission-agent --ros` (or `python -m multi_robot_mission_stack.agent.cli --ros`).
- **Happy paths:** navigate → query; navigate → cancel → query-after-cancel, with cancel success allowing `nav_status` ∈ {`cancelling`, `not_cancellable`} and follow-up query rules as in §K and `scripts/validate_mission_cancel_ros.py` (and facade/CLI counterparts).
- **Invalid-goal cancel:** `goal_id` not registered for the requested `robot_id` → `status: failure`, `nav_status: not_found`, `message: Goal id not found for this robot`.
- **Unknown `robot_id`:** Bridge already returns `failure` with `Unknown robot_id '…'` and `nav_status: not_found` in the cancel path (and analogous patterns elsewhere)—callers distinguish this from invalid-goal cancel via **message**, not only `nav_status`.
- **Registry today:** `MissionBridgeNode` stores `_active_goals: Dict[Tuple[robot_id, goal_id], metadata]` when Nav2 accepts a goal and a non-empty `goal_id` is returned (`nav2_client.Nav2Client.send_goal` uses a UUID string). **Cancel** requires `(robot_id, goal_id) ∈ _active_goals` before calling `Nav2Client.cancel_active_goal()` for that robot. Entries are **not** removed on cancel (comment in code: keep behavior simple).
- **Query today:** `get_navigation_state(robot_id, goal_id)` returns `status: success` and the **request’s** `goal_id` in `data`, even when the pair is **not** tracked; it still calls `Nav2Client.get_goal_state()` for that **robot’s** client (which reflects **at most one** active/completed handle per namespace), and appends a registry disclaimer in `message` when `not tracked`. v1.1 validators use the **matching** `goal_id` after navigate, so this asymmetry is **not** yet frozen as a public contract for arbitrary `goal_id` values.

---

## Goal ownership model

- **Owner key:** The bridge **owns** a navigation goal under the composite key **`(robot_id, goal_id)`**, where `robot_id` is the mission-layer id (`robot1`, `robot2`, …) and `goal_id` is the string returned by the bridge after a successful submit (UUID today).
- **Semantic rule:** Any cancel or authoritative query for a goal **must** be interpreted in terms of that pair. **No other layer** should treat `goal_id` alone as globally actionable without also fixing `robot_id`, except where this note defines a deliberate bridge response (e.g. wrong-robot detection below).
- **Uniqueness (practical):** With UUID `goal_id` generation per `Nav2Client` instance, collisions across robots are negligible; the bridge should still implement ownership checks in terms of **registry content**, not probability.
- **Wrong-robot definition (V2.0.1):** A request is **wrong-robot** when the caller supplies `(robot_id_a, goal_id_g)` but the bridge registry contains **`(robot_id_b, goal_id_g)`** with `robot_id_a ≠ robot_id_b` (i.e. `goal_id` is **known** to the bridge and owned by another robot). This is **distinct** from **unknown goal_id** (no registry entry with that `goal_id` for any robot).

---

## Registry invariants

Proposed invariants for V2.0.1 (implementation may introduce auxiliary indexes; behavior must satisfy these):

1. **Registration:** After a successful navigate that yields non-empty `goal_id`, the bridge **must** contain exactly one logical owner: `(robot_id, goal_id) → metadata` (current metadata: `robot_id`, `goal_id`, `namespace`, `target_pose` as today).
2. **Lookup for cancel:** Cancel **must** first resolve ownership:
   - If `(robot_id, goal_id)` exists → **valid bridge target** for cancel (then delegate to Nav2 as today).
   - Else if `goal_id` exists under some other `robot_id` → **wrong-robot** response (new contract below)—**do not** call cancel on the requested robot’s `Nav2Client` for that goal.
   - Else → **unknown goal for this robot** → preserve §K **invalid-goal cancel** JSON exactly.
3. **Secondary index:** Maintain `goal_id → owning robot_id` (or equivalent) **derived from** `_active_goals` so wrong-robot detection is O(1) without scanning unbounded structures. On removal (if introduced later), update the index consistently.
4. **Lifecycle (V2.0.1 scope):** **No requirement** to remove registry entries on cancel or terminal completion in this milestone **unless** needed for correct wrong-robot vs stale-id behavior. If entries remain after terminal states, document whether wrong-robot detection still applies (recommend: **yes**, until explicit removal policy lands in a later milestone). **Defer** full lifecycle/cleanup policy to **Deferred items** if it would expand scope.
5. **Multi-robot prep:** Invariants are stated so a future orchestrator can ask “who owns `goal_id`?” with a **single** bridge-backed answer—no duplicate registries in scripts or graphs.

---

## Service JSON contract additions

All responses below are described in **bridge logical dict** form; `MissionClient` already maps flat `srv` fields to `status`, `message`, `nav_status` (and `goal_id` where applicable). Facade/CLI inherit the same strings.

### Valid cancel (unchanged from §K / v1.1)

- **Precondition:** `(robot_id, goal_id) ∈ _active_goals`.
- **Response:** `status: success`; `nav_status` from Nav2 client normalization: **`cancelling`** or **`not_cancellable`**; `message` from Nav2 path (may vary). This matches existing `validate_mission_cancel_ros.py` expectations.

### Valid query (unchanged for v1.1-shaped flows)

- **Precondition:** v1.1 validators call query with the `goal_id` returned for that `robot_id` after navigate.
- **Response:** Continue to return `status: success` with `nav_status` from `Nav2Client.get_goal_state()` (`in_progress`, `succeeded`, `cancelled`, `failed`, `unknown`, etc.) and `message` as today. When the pair is not tracked, keep current behavior **unless** V2.0.1 adds a **narrow** clarification (see Deferred items) to avoid mis-read goal_id for a different robot’s Nav2 handle.

### Unknown `goal_id` (for this robot) — **unchanged**

- **Condition:** No `(robot_id, goal_id)` and **no** other `robot_id'` owns `goal_id` in the registry.
- **Cancel response (frozen §K):** `status: failure`, `nav_status: not_found`, `message: Goal id not found for this robot`.

### Wrong-robot cancel — **new in V2.0.1**

- **Condition:** `goal_id` is registered as owned by `robot_id_other`, but the request uses `robot_id != robot_id_other`.
- **Cancel response (target contract):**
  - `status: failure`
  - `nav_status: wrong_robot` *(new enum value; avoids overloading `not_found`)*
  - `message`: stable string **`Goal id belongs to another robot`** (manager-approved; implemented in V2.0.1).

**Rationale:** Today the bridge maps this case to the same JSON as unknown goal (`not_found` + “Goal id not found for this robot”), which is **incorrect** for integrators when the id exists under another robot—orchestration cannot learn ownership from the response.

### Terminal-goal cancel — **named for V2.0.1, implementation may be minimal**

- **Condition:** `(robot_id, goal_id)` is tracked, but Nav2’s completed state for that client indicates the goal is **already terminal** (`succeeded`, `cancelled`, or `failed` per `nav2_client.get_goal_state()` mapping).
- **Target behavior (product clarity):** Cancel should **not** imply a new cancel wave was meaningfully queued if the goal is already done. Prefer:
  - `status: success`
  - `nav_status: not_cancellable`
  - `message`: stable string indicating terminal completion, e.g. **`Goal is already complete`** (exact wording TBD with validator).
- **Note:** Current code may still invoke `cancel_goal_async` in some cases; V2.0.1 **may** short-circuit after reading terminal state to match the target contract, **or** document observed behavior in a validator first—choose the **smaller** diff that still produces a **stable** JSON contract. Full terminal **query** semantics across mismatched `goal_id` remain deferred unless needed for this short-circuit.

### Query for wrong / unknown `goal_id` (non-cancel)

- **V2.0.1:** **No change required** for milestone closure if validators do not yet assert it. **Recommend** follow-up: when `goal_id` is owned by another robot, return `failure` + `wrong_robot` + clear message (and avoid attributing the wrong robot’s Nav2 single-handle state to an arbitrary `goal_id`). Record under Deferred items if not shipped in V2.0.1.

---

## Deferred items

- **Orchestration:** No supervisor, no cross-robot task graphs, no LangGraph changes.
- **Launch / runtime:** No new launch files, no executor/threading redesign beyond what bridge already requires.
- **CLI exit semantics:** Still product-owned; not part of V2.0.1 bridge registry milestone.
- **Registry entry removal / TTL:** Policy for when to drop `(robot_id, goal_id)` after cancel or terminal completion—defer unless wrong-robot or stale behavior **requires** it for correctness.
- **Query with arbitrary `goal_id`:** Full contract matrix (wrong-robot query, unknown query vs Nav2 bleed-through) — defer if cancel + ownership index deliver the orchestration unlock first.
- **ROS `.srv` schema changes:** Prefer **not** adding fields in V2.0.1; use existing `status` / `message` / `nav_status` strings. If `wrong_robot` is rejected by integrators, revisit enum naming once.
- **Facade/CLI parity validators:** After bridge + `MissionClient` validator pass, add facade/CLI validators mirroring v1.1 invalid-goal pattern (separate small PRs acceptable).

---

## Validation plan

1. **Unit-level (optional):** Tests for ownership resolution logic on a plain dict/registry structure (if the repo adds them for the bridge module).
2. **Live ROS (required):** New script `scripts/validate_mission_cancel_wrong_robot_ros.py` (name illustrative):
   - Navigate on **robot1** (or whichever robot the baseline uses), capture `goal_id`.
   - Call `cancel_navigation("robot2", goal_id)` (or second configured id in `_robot_namespaces`).
   - Assert JSON: `failure` / `wrong_robot` / agreed `message`.
3. **Regression:** Re-run existing v1.1 validators (client/facade/CLI) for happy paths and invalid-goal cancel to ensure §K unchanged.
4. **Runbook:** Add a **§K extension** (e.g. “V2.0.1 ownership”) with a short table linking wrong-robot cancel and pointing to this note—**do not replace** §K v1.1 text; **extend** it.

---

## Risks / non-goals

- **Risk — stale registry:** If completed goals remain registered indefinitely, wrong-robot detection may remain true for old ids; orchestration should treat `goal_id` as opaque and time-bounded at the mission layer. Mitigation: document; optional cleanup milestone later.
- **Risk — Nav2Client single handle:** Each namespace still tracks one handle; query semantics for mismatched `goal_id` remain confusing until deferred query work lands.
- **Non-goal:** Redesigning `Nav2Client` into a multi-goal map per robot (not required for V2.0.1 if UUID + bridge registry suffice).
- **Non-goal:** Changing `MissionClient` transport or service names.
- **Non-goal:** Defining global uniqueness of `goal_id` across reboots or processes—only within a running bridge instance.

---

## Relation to §K (mission_system_runbook.md)

- **§K** remains the **operator/integrator** anchor for **v1.1** navigate/query/cancel and invalid-goal cancel.
- **V2.0.1** **adds** explicit ownership and wrong-robot cancel JSON; it **does not invalidate** §K rows—it **narrows** ambiguity where `goal_id` exists under another robot (today misclassified as `not_found`).
- After implementation, the runbook should gain a short **subsection under §K** (or §K.1) summarizing wrong-robot cancel and linking **here** for full rationale.
