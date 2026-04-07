# Mission Control V3.3 — LLM-assisted fact production (design note)

## Purpose

**What V3.3 proves beyond V3.1**

V3.1 established that a **frozen** `blocked_passage` record can move **live** over ROS, ingest into the **existing** belief store, and **block** named-location navigation through the **existing** policy seam with explicit time semantics. The open product question for the next milestone is **provenance of the fact**: *where candidate records come from* when they are not hand-authored or produced by a deterministic stub.

**V3.3** proves **one bounded path** where a **local LLM call** assists in producing a **candidate** `blocked_passage` **record** (same schema as V3.0.1), subject to **mandatory** post-generation validation. Success means: *useful, schema-valid candidates can originate from structured LLM output without changing anything downstream of validation.*

V3.3 does **not** re-ask whether transport, store, or policy work; it asks whether **LLM-assisted generation** can feed the **same** ingest pipeline **safely** and **measurably**.

**Normative I/O for implementation** is frozen in **Frozen V3.3 LLM candidate I/O contract** below. Earlier narrative sections are subordinate to that contract.

---

## Current baseline carried forward

**Frozen from V3.0.1 / V3.1 (non-negotiable for V3.3)**

- **`blocked_passage` schema** — field names, types, literals (`schema_version`, `fact_type`, `verification_status`), UUID v4 rules, provenance shape, `ttl_sec`, TTL + skew semantics, and **no second fact type**.
- **`BlockedPassageBeliefStore`** — validate on ingest, duplicate-ignore by `belief_id`, allowlist, TTL-at-ingest rejection; **no redesign** of storage or merge semantics.
- **Transport** — JSON object in `std_msgs/String` on the agreed topic; **no** new wire format, retries, acks, or multi-hop.
- **Policy hook** — single seam on `navigate_to_named_location`, explicit `now_utc`, frozen blocked outcome; **no** additional mission effects, no LLM in the policy path.
- **Bridge / control plane** — **no** new services or semantic overload for beliefs.
- **Geometry** — **no** map, TF, or costmap mutation from LLM output.
- **Facade time** — **no** hidden wall-clock in policy; any clock policy for **generation** is explicit and documented separately from V3.0.1 policy `now_utc`.

---

## Frozen V3.3 LLM candidate I/O contract (normative)

This section **locks** the pre-model and post-model contracts. **Downstream ingest/transport integration** must not proceed until this contract matches code; local adapter and harness work (V3.3.a/b) are checkpointed separately — see [mission_control_v3_3_checkpoint.md](mission_control_v3_3_checkpoint.md).

### 1. LLM input boundary

The model may receive **only** what is derived from a single JSON object, the **`llm_context`**, with **exactly** these keys and constraints. Prompt text is assembled by **deterministic template** from this object (no ad-hoc prose from operators mixed into the template body beyond the bounded `operator_hint` field).

| Key | Type | Constraints |
|-----|------|-------------|
| `schema_version` | string | **Required.** Must be exactly `"v3.3.llm_context.1"`. |
| `location_ref` | string | **Required.** Non-empty after trim. Must already match caller/config **allowlist** before building `llm_context`. |
| `source_robot_id` | string | **Required.** Non-empty after trim. Mission-layer id (same convention as bridge / `MissionClient`). **Not** model-invented. |
| `nav_goal_status` | string | **Required.** One of: `unknown`, `pending`, `active`, `succeeded`, `failed`, `canceled`. |
| `stall_duration_sec` | number | **Required.** Float `>= 0` and `<= 604800` (7-day cap). |
| `planner_status` | string | **Required.** One of: `unknown`, `idle`, `computing`, `failed`. |
| `lidar_occlusion_proxy` | boolean | **Required.** A **deterministic** binary cue from an existing detector path (not raw lidar). |
| `operator_hint` | string | **Required** key; may be empty string. UTF-8, length after trim **≤ 200**. |

**Rules**

- **No other keys** are valid inside `llm_context`. Implementations **must reject** contexts with unknown keys before calling the model.
- **No** raw sensor arrays, images, point clouds, logs, or multi-turn chat transcripts in V3.3.
- **No** fields whose values are unbounded strings except `operator_hint` (capped at 200).

---

### 2. LLM output boundary

The model must emit **only** a single JSON object, the **`llm_candidate`**, parseable as UTF-8 JSON with **exactly** these keys (extra keys → **reject**; see §4).

| Key | Type | Constraints |
|-----|------|-------------|
| `schema_version` | string | **Required.** Must be exactly `"v3.3.llm_candidate.1"`. |
| `assert_blocked` | boolean | **Required.** If `false`, the invocation is **semantically insufficient** for producing a `blocked_passage` in this path → **reject** (no assembly to final record). |
| `location_ref` | string | **Required** when `assert_blocked` is `true`. Non-empty after trim. Must **exactly equal** `llm_context.location_ref` (byte-for-byte after trim on both sides). Otherwise **reject**. |
| `confidence` | number | **Required** when `assert_blocked` is `true`. Float in `[0.0, 1.0]`, not boolean. |
| `ttl_sec` | number | **Required** when `assert_blocked` is `true`. Float `> 0.0` and `<= 86400.0` (24 h cap; deployments may tighten further in code). |
| `sensor_class` | string | **Required** when `assert_blocked` is `true`. Non-empty after trim, length `<= 64`, characters **only** `[a-z0-9_]` (lowercase snake token). |
| `rationale_short` | string | **Optional.** If present, length after trim `<= 280`. Used **only** for logging/evaluation; **stripped** before final record assembly (not a `blocked_passage` field). |

**Locked decision:** the model **does not** emit the full final `blocked_passage` record. That prevents hallucinated `belief_id`, `fact_type`, `verification_status`, `timestamp_utc`, and `provenance.observation_id`, and keeps the attack surface minimal. **Full-record LLM output is out of scope for V3.3** unless a future revision of this contract explicitly supersedes this section.

---

### 3. Code-owned final fields

After a successful `llm_candidate` parse and gate checks, **deterministic code** builds the final record for `validate_blocked_passage_record` / ingest. The following **final** `blocked_passage` fields are **always** set by code, **never** taken from model text:

| Final field | Source |
|-------------|--------|
| `schema_version` | Constant `"v3.0.1"`. |
| `fact_type` | Constant `"blocked_passage"`. |
| `belief_id` | New UUID v4 generated in code **after** successful candidate acceptance. |
| `source_robot_id` | Copied from **`llm_context.source_robot_id`** (not from `llm_candidate`). |
| `timestamp_utc` | RFC 3339 string from an **explicit** code clock policy at assembly time (documented per deployment; no hidden default inside validators). |
| `confidence` | Copied from `llm_candidate` (already range-checked). |
| `location_ref` | Copied from `llm_candidate` **only after** equality check with `llm_context.location_ref`. |
| `provenance.sensor_class` | Copied from `llm_candidate.sensor_class`. |
| `provenance.observation_id` | New UUID v4 generated in code (correlation / log row id). |
| `ttl_sec` | Copied from `llm_candidate` (already bounds-checked). |
| `verification_status` | Constant `"unverified"`. |

**`rationale_short`** is **not** copied into the final record.

---

### 4. Rejection behavior

All rejections: **no** `BlockedPassageBeliefStore.ingest`, **no** transport publish, **no** policy side effects. Implementations **may** append one structured log line (event + reason code); **no** silent fallback to stub or alternate model in the default path.

| Condition | Outcome |
|-----------|---------|
| `llm_context` fails schema/key/constraint checks before the model | **Reject** — `context_invalid`. |
| Model output is not valid UTF-8 JSON, or JSON is not a single object | **Reject** — `candidate_malformed`. |
| Unknown keys in `llm_candidate`, or missing required keys for the declared `schema_version` | **Reject** — `candidate_missing_or_extra_keys`. |
| `schema_version` ≠ `v3.3.llm_candidate.1` | **Reject** — `candidate_schema_mismatch`. |
| `assert_blocked` is `false` | **Reject** — `candidate_semantically_insufficient` (intentional “do not assert” from model). |
| `assert_blocked` is `true` but `location_ref` ≠ `llm_context.location_ref`, or any numeric/string bound fails | **Reject** — `candidate_bounds_or_location_mismatch`. |
| After assembly, `validate_blocked_passage_record` returns errors | **Reject** — `final_schema_reject` (include validator messages in log only; **no** field repair or LLM retry in the default path). |

**No silent repair:** code may only **inject** the code-owned fields in §3 and **map** validated candidate fields; it **must not** “fix” invalid model values (e.g. clamping confidence outside stated policy) unless a **separate** explicitly named mode is added in a later contract revision.

---

### 5. Evaluation fixture format

Fixtures are **JSON files** (or JSON objects embedded in pytest) using this shape:

```json
{
  "fixture_id": "string",
  "llm_context": { },
  "simulated_model_output": "string",
  "expect": {
    "parse": "accept|reject",
    "reject_reason": "null|string",
    "final_validation": "pass|fail|skip"
  }
}
```

| Field | Meaning |
|-------|---------|
| `fixture_id` | Stable id for regression lists. |
| `llm_context` | Full object obeying §1. |
| `simulated_model_output` | **Raw string** exactly as if returned by the model (may include whitespace; parser strips once). Used for **non-LLM** unit tests of the parser/assembler. |
| `expect.parse` | `accept` if, after JSON parse + §2 checks, assembly is allowed; `reject` otherwise. |
| `expect.reject_reason` | If `reject`, substring or enum matching one of §4 reason tokens (implementation maps to exact tokens); `null` if `accept`. |
| `expect.final_validation` | `pass` if assembled dict must pass `validate_blocked_passage_record`; `fail` if it must not; `skip` if parse rejects before assembly. |

**Optional** (same file, for richer eval):

```json
  "notes": "human-readable"
```

Same pipeline for stub vs LLM tests: **fixture** drives `llm_context` + `simulated_model_output`; downstream is always **assemble → validate_blocked_passage_record → (optional) ingest in integration tests**.

---

## Example candidate I/O

### Allowed `llm_context` (concrete)

```json
{
  "schema_version": "v3.3.llm_context.1",
  "location_ref": "base",
  "source_robot_id": "robot1",
  "nav_goal_status": "active",
  "stall_duration_sec": 42.5,
  "planner_status": "computing",
  "lidar_occlusion_proxy": true,
  "operator_hint": "corridor feels blocked ahead"
}
```

### Allowed `llm_candidate` (concrete)

```json
{
  "schema_version": "v3.3.llm_candidate.1",
  "assert_blocked": true,
  "location_ref": "base",
  "confidence": 0.82,
  "ttl_sec": 180.0,
  "sensor_class": "lidar_occlusion",
  "rationale_short": "Stall plus planner churn aligns with prior blockage pattern."
}
```

### Rejected model output (concrete)

`location_ref` hijack (must reject before ingest):

```json
{
  "schema_version": "v3.3.llm_candidate.1",
  "assert_blocked": true,
  "location_ref": "warehouse_door",
  "confidence": 0.9,
  "ttl_sec": 120.0,
  "sensor_class": "lidar_occlusion"
}
```

Reason: **`candidate_bounds_or_location_mismatch`** (does not equal `llm_context.location_ref` `"base"`).

---

## Allowed LLM role

**Exactly what the LLM is allowed to do**

- Consume prompt text built **only** from a validated **`llm_context`** (§1).
- Emit **one** raw string that parses to **`llm_candidate`** (§2) per invocation.

**Exactly what the LLM is not allowed to do**

- Emit the **full** final `blocked_passage` JSON (§2, locked).
- **Write** to the belief store, publish on the transport topic, or call navigation / bridge / policy APIs **directly**.
- Introduce **keys** or **fact types** outside §1–§2.
- **Bypass** assembly + `validate_blocked_passage_record` (§4).

---

## Input to the LLM (summary)

See **§1. LLM input boundary**. No additional “local observation” channels are permitted in V3.3 beyond that object and the deterministic template.

---

## Output of the LLM (summary)

See **§2. LLM output boundary** and **§3. Code-owned final fields**. Assembly is **code-only** after candidate acceptance.

---

## Validation boundary

- After assembly, **`validate_blocked_passage_record`** (frozen V3.0.1) is **mandatory**. Failure → §4 `final_schema_reject`.
- Optional deployment caps (e.g. stricter `ttl_sec` max) are **code** checks on `llm_candidate` **before** assembly, not silent repair.

---

## Decision boundary

- The **LLM** produces **`llm_candidate`** only.
- **Code** owns assembly (§3), validation, and only then **`ingest`** / **encode + publish** on existing paths.
- The **belief store** and **policy** are unchanged.

---

## Evaluation plan

- Use **§5** fixtures for parser/assembler regression; compare to **stub**-built records through the **same** `validate_blocked_passage_record` → ingest → policy tests.
- Metrics: validity rate, precision/recall vs ground truth when available, latency/cost, **zero** invalid ingests.

---

## Risks / non-goals

**Risks**

- Hallucination within allowed keys — mitigated by strict bounds, `location_ref` tie to context, and final validator.
- Prompt injection via `operator_hint` — mitigated by length cap and no execution of model output.

**Non-goals**

- Second fact type; schema drift; NL on-wire; LLM in policy/ingest; full-record model output in V3.3; silent repair loops.

---

## What was removed or constrained (vs earlier draft)

- **Vague “local observations”** replaced by the **closed key set** `llm_context` (§1).
- **Full-record LLM output** explicitly **ruled out** for V3.3; only **`llm_candidate`** (§2).
- **`assert_blocked`** added so the model can abstain without inventing junk fields.
- **`location_ref` equality** to context is **mandatory** when asserting — blocks hijack.
- **`sensor_class` character set** capped to lowercase snake token.
- **Rejection reasons** enumerated; **no ingest/publish** on any reject.
- **Fixture format** is **JSON**, single `simulated_model_output` string for deterministic tests without a model.
- **`source_robot_id` and `timestamp_utc`** are **never** model outputs; **observation_id** is **never** model output.

---

## Related documents

- [mission_control_v3_0_1_checkpoint.md](mission_control_v3_0_1_checkpoint.md) — frozen `blocked_passage` semantic and ingest/policy base.
- [mission_control_v3_1_checkpoint.md](mission_control_v3_1_checkpoint.md) — live single-hop demo closure.
- [mission_control_v3_3_checkpoint.md](mission_control_v3_3_checkpoint.md) — paused V3.3 line closure (local gates, adapter, harness).
- [mission_control_v3_0_1_semantic_fact_exchange.md](mission_control_v3_0_1_semantic_fact_exchange.md) — normative record contract.
