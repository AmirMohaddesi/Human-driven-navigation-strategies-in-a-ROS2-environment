# Mission Control V4.0 — Bounded real-LLM launch-topology smoke (checkpoint)

## Strategic question

Can the **V3.9 launch-based** handoff + witness topology run **one bounded real-model** path while keeping the same request envelope, explicit clocks, deterministic gates, V3.0.1 transport reuse, and **advisory-only** witness outcome?

## Purpose

Increase **model-side** runtime realism without broadening authority, APIs, schemas, or policy semantics.

## Audit (V4.0)

- **Launch topology:** Reuse `semantic_handoff_advisory_v39.launch.py` unchanged. Real vs fake is selected only by the JSON field `use_deterministic_fake_adapter` (`false` → existing `OpenAiChatCompletionsAdapterV33` in `execute_semantic_production_handoff_v35`).
- **True blockers:** None in code. **Operational:** `OPENAI_API_KEY` must be present in the **handoff node’s** environment for a successful provider call. Optional `OPENAI_BASE_URL` for OpenAI-compatible endpoints. No launch file or srv changes required.

## Already proven (V3.9 and earlier)

- Fake-adapter path on the same launch topology, transport mirror, witness ingest, advisory block/control.

## Newly proven by V4.0

- **Failure path (no secrets in launched processes):** `use_deterministic_fake_adapter: false` with `OPENAI_API_KEY` **stripped** from the launch subprocess env → service returns `outcome=adapter_failure`, `adapter_reason=adapter_provider_error`, bounded detail (no ingest, no mirror, witness unchanged).
- **Success path (opt-in):** With `RUN_V4_LLM_SMOKE=1` and `OPENAI_API_KEY` set, automated pytest can observe `ingest_stored` and witness **blocked** outcome (subject to model obeying the strict JSON contract; gate stack unchanged).

## Intentionally not proven

- Mission bridge integration, prompt tuning, provider bake-off, product readiness, guaranteed model compliance on all prompts.

## Prerequisites (success-path manual or opt-in test)

```bash
export OPENAI_API_KEY="sk-..."   # required for real call
# optional: export OPENAI_BASE_URL="https://..."  # compatible chat/completions API
```

Start launch from a shell that **exports** these variables so `semantic-production-handoff-v35` inherits them.

## Build / source

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ws
colcon build --packages-select multi_robot_mission_stack_interfaces multi_robot_mission_stack
source install/setup.bash
```

## Launch (unchanged V3.9)

```bash
ros2 launch multi_robot_mission_stack semantic_handoff_advisory_v39.launch.py
```

## Service calls

Use a **wall-aligned** envelope (same shape as V3.9 tests): set `assembly_timestamp_utc_iso` and `ingest_now_utc_iso` to **now** (UTC) so witness ingest TTL remains valid.

**Real adapter:**

```json
{
  "llm_context": { ... },
  "assembly_timestamp_utc_iso": "<now UTC Z>",
  "ingest_now_utc_iso": "<now+1s UTC Z>",
  "use_deterministic_fake_adapter": false
}
```

Call `produce_semantic_blocked_passage_v35` with `json_request` set to that JSON string.

Then call `query_advisory_named_nav_v39` with `robot_id`, `location_name` matching `llm_context` (e.g. `base`), and `decision_now_utc_iso` within TTL after ingest.

**Bounded failure demo (no key):** start launch from a shell **without** `OPENAI_API_KEY`; same request with `use_deterministic_fake_adapter: false` → `adapter_failure` / missing key detail.

## Automated tests (repo root)

**Default CI-friendly (no API key, no network):**

```bash
cd /path/to/HDNS
source /opt/ros/humble/setup.bash
source /path/to/ws/install/setup.bash
python3 -m pytest tests/test_semantic_real_llm_smoke_v40_ros.py::test_v40_real_adapter_missing_key_bounded_adapter_failure -v
```

**Opt-in real-model smoke (network + cost):**

```bash
export RUN_V4_LLM_SMOKE=1
export OPENAI_API_KEY="sk-..."
python3 -m pytest tests/test_semantic_real_llm_smoke_v40_ros.py::test_v40_real_llm_ingest_stored_witness_blocks -v
```

## Recorded evidence (automated, no provider key)

Example service JSON response shape when the handoff process has **no** `OPENAI_API_KEY` and `use_deterministic_fake_adapter: false`:

```json
{
  "outcome": "adapter_failure",
  "adapter_reason": "adapter_provider_error",
  "adapter_detail": "missing API key (pass api_key= or set OPENAI_API_KEY)"
}
```

Verified by `tests/test_semantic_real_llm_smoke_v40_ros.py::test_v40_real_adapter_missing_key_bounded_adapter_failure` against the V3.9 launch file (launch subprocess env strips OpenAI variables).

## Related

- [mission_control_v3_9_checkpoint.md](mission_control_v3_9_checkpoint.md)
