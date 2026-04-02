# Mission Control V2.0.2 — Terminal-goal cancel and query semantics (design note)

## Purpose

Close the **remaining control-plane gaps** called out in **§K** ([mission_system_runbook.md](mission_system_runbook.md)) and deferred from **V2.0.1** ([mission_control_v2_0_1_registry_ownership.md](mission_control_v2_0_1_registry_ownership.md)):

1. **Terminal-goal cancel:** stable JSON when `(robot_id, goal_id)` is **registry-tracked** but Nav2’s single handle for that namespace is **already terminal** (succeeded / cancelled / failed).
2. **Query semantics:** explicit behavior for **tracked vs untracked** `goal_id` (and, where practical, **wrong-robot query**) so integrators do not misread **Nav2’s one-handle state** as applying to an arbitrary `goal_id`.

This note is **design only** until approved. It does **not** expand orchestration (V2.1 stays checkpointed), redesign launch/runtime, change [mission_control_cli_policy.md](mission_control_cli_policy.md), or add mission contract types.

---

## Current baseline carried forward

**Frozen by v1.1 + validators (§K, §E):**

- Happy paths: **navigate → query**; **navigate → cancel → query-after-cancel** with cancel `nav_status` ∈ {`cancelling`, `not_cancellable`} and follow-up query expectations as in §K and `scripts/validate_mission_cancel_ros.py` (plus facade/CLI counterparts).
- **Invalid-goal cancel:** bogus `goal_id` for robot → `failure` / `not_found` / `Goal id not found for this robot` (`validate_mission_cancel_invalid_goal_ros.py` and facade/CLI mirrors).

**Frozen by V2.0.1 + validators:**

- **Wrong-robot cancel:** `failure` / `wrong_robot` / `Goal id belongs to another robot` when `goal_id` is owned by another robot (`validate_mission_client_ownership_cancel_ros.py`, coordinator/facade/CLI ownership validators).
- **Registry model:** `_active_goals[(robot_id, goal_id)]` and `_goal_id_owner[goal_id]` in `mission_bridge_node.py`; entries are **not** removed on cancel (behavior kept simple in code today).

**Observed implementation (not yet contract-frozen for terminal/query):**

- **Cancel (owned_active):** `MissionBridgeNode.cancel_navigation` always calls `Nav2Client.cancel_active_goal()` after ownership checks; there is **no** short-circuit on Nav2 terminal state before issuing cancel ([mission_bridge_node.py](../../src/multi_robot_mission_stack/multi_robot_mission_stack/bridge/mission_bridge_node.py)).
- **Nav2Client:** One `_goal_handle` per namespace; `get_goal_state()` reflects **that** handle only ([nav2_client.py](../../src/multi_robot_mission_stack/multi_robot_mission_stack/bridge/nav2_client.py)). `cancel_active_goal()` returns `not_cancellable` only when there is **no** handle; if the handle exists it always sends `cancel_goal_async` and returns `cancelling`.
- **Query:** `get_navigation_state` always returns `status: success` for a configured `robot_id` and non-empty `goal_id`, embeds the **request’s** `goal_id` in `data`, and appends a **registry disclaimer** in `message` when the pair is **not** in `_active_goals`—but still reads Nav2 state from that robot’s client.

**§K explicit non-freeze (today):** “Wrong-robot cancel and **terminal-goal** cancel/query behavior are **not** frozen by the current validator set unless separately validated later.”

---

## Terminal-goal cancel contract

**Problem:** For a **tracked** `(robot_id, goal_id)`, callers need a **stable** response when the navigation goal is **already finished** in Nav2, without implying a meaningful new cancel wave.

**Target contract (V2.0.2):**

- **Precondition:** `_classify_cancel_target` would return **owned_active** (pair in `_active_goals`) **and** Nav2 state for that robot’s client is **terminal** (`succeeded`, `cancelled`, or `failed` per `Nav2Client.get_goal_state()` normalization).
- **Response (product-facing JSON, bridge logical dict):**
  - `status`: **`success`**
  - `nav_status`: **`not_cancellable`**
  - `message`: **`Goal is already complete`** (frozen; live check: `scripts/validate_mission_cancel_terminal_goal_ros.py`).
- **Behavior:** **Do not** call `cancel_goal_async` in this branch (short-circuit after reading terminal state).

**Out of scope for this subsection:** Registry cleanup after terminal (remains a separate policy decision; V2.0.2 may still **not** remove `_active_goals` entries if wrong-robot and audit behavior remain acceptable).

**Relationship to v1.1 cancel validators:** Existing scripts focus on **active** cancel; V2.0.2 adds a **dedicated** live check (navigate → wait or drive until terminal → cancel same `goal_id`) so §K can be extended without breaking happy-path expectations.

---

## Query semantics

**Problem:** `get_navigation_state(robot_id, goal_id)` returns `status: success` even when the pair is **untracked**, while `nav_status` comes from Nav2’s **single** current/previous handle. That can mislead callers who assume `nav_status` describes **their** `goal_id`.

**Design axes (V2.0.2 should pick minimal stable rules):**

| Case | Registry | Intended semantics (target) |
|------|----------|-----------------------------|
| **Tracked, handle matches mission expectation** | `(robot_id, goal_id) ∈ _active_goals` | Keep **success** + Nav2-derived `nav_status` for that handle; message may clarify terminal vs in-progress. |
| **Untracked, unknown goal** | No `(robot_id, goal_id)`, and `goal_id` not in `_goal_id_owner` OR owned by this robot but pair missing (edge: define explicitly) | Prefer **failure** or **success with nav_status `not_found` / `unknown`**—**one** rule, documented; avoid silently attributing Nav2 state to the wrong id. |
| **Wrong-robot query** | `goal_id` owned by `robot_id_other ≠ robot_id` | Align with cancel clarity: e.g. `failure` + `nav_status: wrong_robot` + stable `message` (V2.0.1 recommended this as follow-up). |

**Non-goals for first V2.0.2 slice (unless blocker):**

- Multi-goal history per robot inside Nav2 (still one handle in `Nav2Client`).
- Changing `.srv` field layout (prefer existing `status` / `message` / `nav_status` strings per V2.0.1 discipline).

---

## Registry vs Nav2 truth model

**Bridge registry (`_active_goals`, `_goal_id_owner`):**

- **Mission-layer record** of goals the bridge **submitted** and registered with a non-empty `goal_id`.
- **Persists** across cancel today (entries not removed); used for **wrong_robot** and **owned_active** classification.
- **Truth for:** “Did **this** bridge instance register this `(robot_id, goal_id)`?” and “Who owns `goal_id`?”

**Nav2 client (`Nav2Client`):**

- **Runtime truth** for **at most one** outstanding/completed handle per namespace in this wrapper.
- **Truth for:** “What is the state of the handle this client is holding?” **Not** automatically “what is the state of an arbitrary `goal_id` string?”

**V2.0.2 rule of thumb:** Service responses must **bridge** these two: when registry and handle **align** (typical v1.1 flows), behavior stays as today. When they **diverge** (untracked query, terminal cancel, stale id), responses must be **explicit** (`not_cancellable`, `not_found`, `wrong_robot`, stable messages) so upper layers do not infer false ownership or false Nav2 alignment.

---

## Deferred items

- **Orchestration / Layer B** beyond what §K and existing coordinator APIs already specify.
- **Launch/runtime** and executor threading redesign.
- **Registry TTL / eviction** policy after terminal or cancel (document tradeoffs; implement only if required for correct V2.0.2 contracts).
- **Multi-goal Nav2 map** per robot inside the bridge (larger refactor).
- **CLI policy** changes ([mission_control_cli_policy.md](mission_control_cli_policy.md)); JSON remains primary for semantics.
- **New mission types** and LangGraph expansion.

---

## Validation plan

1. **Live ROS — terminal cancel:** New script (e.g. under `scripts/validate_mission_*_terminal_cancel_ros.py`): navigate → observe terminal (poll `get_navigation_state` or timeout policy as agreed) → **cancel same `goal_id`** → assert **success** + **not_cancellable** + agreed **message**; MissionClient path minimum; facade/CLI parity optional follow-up.
2. **Live ROS — query matrix (incremental):** Small focused scripts or rows in an existing pattern for: untracked `goal_id` query; wrong-robot query (if shipped in same milestone).
3. **Regression:** Re-run §E validators: `validate_mission_client_ros.py`, facade navigate/cancel, `validate_mission_cancel_invalid_goal_ros.py` (+ facade/CLI), ownership cancel scripts—§K + V2.0.1 unchanged except where §K is **extended** with new rows for V2.0.2.
4. **Runbook:** Add a **§K** subsection (e.g. “V2.0.2 terminal/query”) with a short table linking to **this note**—**extend** §K, do not replace v1.1 text.

---

## Risks / non-goals

- **Risk — stale registry + terminal short-circuit:** Terminal cancel may succeed with `not_cancellable` while registry still holds the pair; wrong_robot may still apply later for the same `goal_id`. Document for orchestrators; optional cleanup milestone separate.
- **Risk — single Nav2 handle:** Query for an old `goal_id` after a new navigate may read the **new** goal’s state; mitigated by stricter **untracked** / **wrong_robot** query rules.
- **Non-goal:** Redefining V2.0.1 wrong-robot **cancel** JSON.
- **Non-goal:** Global `goal_id` uniqueness across reboots (bridge instance scope only).
