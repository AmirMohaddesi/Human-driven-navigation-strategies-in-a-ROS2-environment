# Mission Control V3.0.1 — Structured semantic fact exchange over a constrained swarm link (design note)

## Purpose

**What this milestone proves**

Robots with **independent** classical stacks (localization, planning, mission bridge) can exchange **one** kind of **structured, typed fact** over a **bandwidth- and latency-constrained** inter-robot link, ingest it as **semantic belief** (not geometric truth), and use it to make **one** additional **high-level** decision—without changing Nav2 semantics, bridge §K contracts, or introducing natural language as the protocol.

**Why it matters**

Real swarms are **partially observable** and **asymmetric**: robot A may see a blockage robot B cannot yet see. Today, absent sharing, B acts only on its own sensors. V3.0.1 establishes a **minimal, inspectable** pattern: **encode observation → transmit record → ingest as belief → bounded policy hook**—the template for later richer semantics without a swarm-wide knowledge graph or free-form chat.

---

## Current baseline carried forward

**What stays classical**

- **Geometry, localization, mapping, and motion:** existing ROS 2 / Nav2 / SLAM / TF assumptions; no redefinition of map frames or planning algorithms in this milestone.
- **Mission control plane:** navigate / query / cancel and registry semantics (**§K**, V2.0.x) remain **unchanged**; V3.0.1 does not replace or overload bridge services for navigation proof.
- **LangGraph / mission graphs:** no broad expansion; any hook should be a **thin adapter** or **policy callback** if needed, not a new graph taxonomy.

**What stays unchanged from the current project**

- Repository layout, launch philosophy, and validator discipline for the control plane.
- **No** new mission contract types required for V3.0.1 unless explicitly added later as a separate, small extension.
- **Protocol:** **structured records only** (e.g. JSON or protobuf-like field sets)—**no** free-form natural language as the on-wire or on-log contract.

---

## First fact type

**Chosen type: `blocked_passage`**

**Why this one (and not something else)**

- **Observable and falsifiable:** a passage can be reported blocked with a **spatial anchor** that is still classical—the **mission `location_name` string** (see frozen `location_ref`)—without claiming global metric ground truth.
- **Safety-relevant under constraints:** under limited comms, a single “do not assume this corridor is traversable” signal is **high leverage** and **easy to simulate** in tests (inject fact vs baseline).
- **Narrow taxonomy:** one type avoids premature ontology design; alternatives like `human_present` or `hazard` blur boundaries without adding unique value for the **first** milestone.

---

## Frozen schema contract (V3.0.1) — `blocked_passage` only

This section is the **authoritative** contract for the first semantic fact. **No transport, ROS `.msg`, or code** is implied by this freeze—only the logical record.

### Field table

| Field | Type / constraint | Meaning |
|--------|-------------------|---------|
| `schema_version` | string, exactly `"v3.0.1"` | Parser version gate. |
| `belief_id` | string, UUID **v4** (RFC 4122 canonical lowercase hex + hyphens) | **Globally unique id for this assertion instance** (one emission = one `belief_id`). Used for deduplication, logging, and correlating TTL/expiry to a single record. **Not** reused across re-sends; a “refresh” is a **new** record with a **new** `belief_id`. |
| `fact_type` | string, exactly `"blocked_passage"` | Only allowed fact type in this milestone. |
| `source_robot_id` | string, non-empty | Mission-layer id (e.g. `robot1`, `robot2`), **same convention** as bridge / `MissionClient` `robot_id`. |
| `timestamp_utc` | string, **RFC 3339** (`YYYY-MM-DDTHH:MM:SS.sssZ` or explicit offset) | **Assertion time at source** (when A decided to emit the fact), **not** receive time at B. |
| `confidence` | float, **closed interval `[0.0, 1.0]`** | **Source-reported strength** of the claim. **Interpretation (V3.0.1):** ordinal self-report only—**not** a calibrated probability, not comparable across heterogeneous sensors. **Monotonic use:** higher values mean “A asserts blockage more strongly”; policy hooks **may** threshold on `confidence` only if documented; default policy **need not** branch on it. Out-of-range values are **invalid** records. |
| `location_ref` | string, non-empty | **Exactly one representation for the whole milestone:** the **mission named-location identifier**—the same string that would be passed as `location_name` to `navigate_to_named_location` for this deployment (key into `named_locations.yaml` / operator naming). **No** grid indices, **no** topological ids, **no** wrapper object—**only this string**. |
| `provenance` | object (fixed keys below) | Evidence summary; **no** raw sensor payloads. **Both** keys are **required** in V3.0.1: `sensor_class` (string, non-empty, e.g. `lidar_occlusion`), `observation_id` (string, UUID v4) linking to an internal (local) log row if present. |
| `ttl_sec` | float, **`> 0.0`** | Duration in **seconds** after `timestamp_utc` for which the fact may influence B’s **allowed** behavior (see **TTL rule**). |
| `verification_status` | string, exactly **`unverified`** | **Minimal enum for V3.0.1:** only this literal is **valid** on the wire and at first ingest. **Corroboration, contradiction, or peer review** are **out of scope** for this milestone; reserved for later schema versions. Senders and receivers **must** set `unverified`. |

### TTL rule (normative)

1. **Active window:** A fact is **active** for robot B iff, at B’s evaluation time `t_B`,  
   `t_B ≤ parse(timestamp_utc) + ttl_sec + ε_skew`,  
   where `ε_skew` is a small non-negative bound agreed for the deployment (default recommendation: **0.5 s**) to absorb clock skew. If the inequality fails, the fact is **inactive** (expired).

   **Executable contract (code):** The reference implementation uses **`ε_skew = 0.5` s** (`TTL_SKEW_ALLOWANCE_SEC`) and requires callers to pass an explicit timezone-aware **`now_utc`** for every ingest/query that depends on time (no hidden wall-clock calls in public APIs), so tests and production both anchor on the same UTC rule.
2. **On expiry:** The fact **must not** be used to alter any **allowed decision** (below). B **must stop** treating that `belief_id` as grounds to block or deprioritize `location_ref`.
3. **History / logs:** An **append-only** audit log **may** retain expired records (including `belief_id`, `timestamp_utc`, `ttl_sec`) for debugging and metrics. **Inactive** facts in such a log **must not** feed the live policy path. Optional in-memory stores **should** mark or purge inactive entries by implementation choice, but **policy** must only consult **active** facts.

### Allowed decision influence on robot B (exactly one)

**Single allowed influence (V3.0.1):** When robot B is about to call **`navigate_to_named_location` with a given `location_name`**, if there exists **any active** `blocked_passage` fact (per TTL rule) whose `location_ref` **string-equals** that `location_name`, then B **must not** issue navigation to that **same** `location_name` as the **chosen** target for that decision step, and **must** take the **configured fallback** instead: either navigate to a **different** named location from an explicit operator/config allowlist **or** return a **structured non-success outcome** (mission-layer JSON / return dict) whose meaning is **“target blocked by peer belief”**—without mutating the map, TF, or Nav2 costmaps, and **without** canceling goals already accepted earlier.

No other behavioral effects (exploration bias, global replanning, multi-goal reordering beyond this single gate) are in scope for V3.0.1.

**Structured outcome (no fallback):** Reference dict keys: `outcome` = `navigation_target_blocked`, `schema_version` = `v3.0.1`, `reason_code` = `blocked_passage_peer_belief`, `requested_location_name`, `active_belief_ids` (list of strings). Implemented as ``make_blocked_by_peer_belief_outcome`` in ``multi_robot_mission_stack.semantic.blocked_passage_v301``.

### Canonical example instance

Valid JSON example (pretty-printed for reading; on-wire may be minified):

```json
{
  "schema_version": "v3.0.1",
  "belief_id": "a1b2c3d4-e5f6-4789-a012-3456789abcde",
  "fact_type": "blocked_passage",
  "source_robot_id": "robot1",
  "timestamp_utc": "2026-04-02T18:30:00.000Z",
  "confidence": 0.85,
  "location_ref": "base",
  "provenance": {
    "sensor_class": "lidar_occlusion",
    "observation_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479"
  },
  "ttl_sec": 120.0,
  "verification_status": "unverified"
}
```

*(Illustrative values; `location_ref` must match a real named location in the deployment.)*

---

## Production path

**Robot A: local observation → structured fact**

1. **Trigger:** A deterministic, inspectable detector (e.g. “no progress + sustained obstacle proxy” or a **lab-instrumented** flag)—**not** NL reasoning.
2. **Anchor:** Set **`location_ref`** to the **mission named-location string** (see **Frozen schema contract**).
3. **Emit:** Allocate a **new** `belief_id` (UUID v4); build the full record per the field table; set `verification_status` to **`unverified`**; set `timestamp_utc` at assertion time.
4. **Transmit (later milestone slice):** Serialized record on the constrained link—**not** part of this schema-only step.

---

## Belief ingestion path

**Robot B: store as shared belief, not ground truth**

1. **Validate** full schema: required fields, types, ranges, literals (`schema_version`, `fact_type`, `verification_status`), UUID formats, `location_ref` non-empty string.
2. **Reject** (do not ingest for policy) if `source_robot_id` is not in B’s config allowlist, or if the record is **already inactive** at receive time per the **TTL rule**, or if validation fails.
3. **Store** under **`belief_id`** (primary key). **Duplicate** `belief_id`: second ingest is **ignored** (no overwrite). Secondary index **may** map `(location_ref)` → set of active `belief_id`s for fast lookup at the **navigate_to_named_location** gate.
4. **Do not** inject into map server, static costmap, or TF as authoritative geometry.
5. **`verification_status`** remains **`unverified`** for V3.0.1; no receiver updates to other values in this milestone.

---

## Allowed influence on behavior (reference)

The **only** allowed mission-layer effect is defined **normatively** under **Allowed decision influence on robot B** in the **Frozen schema contract** above. **Not allowed:** hard override of in-flight Nav2 goals, map mutation, multi-hop relay, or any influence beyond that single gate.

---

## Success metric

**Primary (measurable)**

- **Scenario:** Scripted blockage visible only to A; B must route or replan to a secondary named goal.
- **Baseline:** No exchange—B attempts primary goal and fails or stalls beyond a timeout **T**.
- **Treatment:** With V3.0.1 exchange—B **within T** selects alternate behavior (successful navigation to secondary goal **or** explicit structured “avoidance due to peer fact” within **N** planner calls).
- **Report:** Binary pass rate over **K** trials + **time-to-safe-decision** (wall clock).

**Secondary (engineering)**

- **Inspectability:** 100% of on-wire payloads parseable by a CI schema check; zero NL fields in protocol.

---

## Deferred items

- **Transport** (topics, `.msg`, serialization) and on-wire framing.
- Additional `fact_type` values or a **taxonomy**.
- **Alternate `location_ref` shapes** (grid cell, topological id, pose, polygon)—**forbidden** until a new schema version.
- **`verification_status` values other than `unverified`** (corroboration, contradiction, operator sign-off).
- **Swarm-wide** or **multi-hop** relay, CRDTs, or distributed knowledge graph.
- **Natural-language** operator chat or LLM-on-the-wire.
- **Global** semantic map fusion, vector embeddings as interchange, or ontology reasoning.
- **Deep** LangGraph redesign; **changes** to bridge cancel/navigate contracts.
- **Calibration** of `confidence` as true probability; clock sync tighter than `ε_skew` without an explicit follow-on note.

---

## Related documents

- [mission_control_v3_0_1_checkpoint.md](mission_control_v3_0_1_checkpoint.md) — **milestone closure:** what was proven, accepted tests, out of scope; next work is a new milestone.
- [mission_system_runbook.md](mission_system_runbook.md) — control plane §K / validators (unchanged by this note).
- [mission_control_v2_0_2_terminal_query_semantics.md](mission_control_v2_0_2_terminal_query_semantics.md) — prior control-plane milestone (orthogonal substrate).
