# Mission Contract v1

Layer B team named-navigation missions: canonical input spec, stable inspect/execute/summary payloads, and lightweight validators. Runtime behavior is unchanged; v1 tags document and enforce shape only.

## Mission Spec Schema

**Single spec (dict)**

| Field | Required | Constraints |
|-------|-----------|---------------|
| `version` | yes | Must be `"v1"`. |
| `mode` | yes | `"sequence"` or `"parallel"` (whitespace trimmed, case-insensitive). |
| `steps` | yes | List of step dicts. |
| `options` | no | If present, must be a dict (same optional keys as runtime: timeouts, `continue_on_failure`, `max_workers`, `bridge_node_name`). |

**Each step (dict)**

| Field | Required | Constraints |
|-------|-----------|---------------|
| `robot_id` | yes | Non-empty after `str(...).strip()`. |
| `location_name` | yes | Non-empty after `str(...).strip()`. |

**Parallel mode:** duplicate `robot_id` values in one spec (after trim) are invalid for the contract.

**Batch:** a batch is a list of specs; each element must satisfy the single-spec contract. Validate with `validate_team_named_mission_specs_contract`.

**Note:** Runtime entrypoints such as `run_team_named_mission_spec` still accept legacy specs without `version`; contract validators are the formal gate for v1-compliant documents.

## Execution Contract

Stable return shapes (all include additive `"version": "v1"`):

- **`execute_team_named_mission_spec`:** top-level keys include `ok`, `version`, `mode`, `overall_outcome`, `run_result`, `execution_summary`, `summary`.
- **`execute_team_named_mission_spec_checked`:** `ok`, `version`, `mode`, `overall_outcome`, `execution` (nested execute payload), `validation`, `summary`.
- **`execute_team_named_mission_specs`:** `ok`, `version`, `overall_outcome`, `run_result`, `execution_summary`, `summary` (no top-level `mode`).
- **`execute_team_named_mission_specs_checked`:** `ok`, `version`, `overall_outcome`, `execution`, `validation`, `summary`.

Validate payloads with `validate_team_named_mission_execution_contract` (single) or `validate_team_named_mission_specs_execution_contract` (batch). These check required keys and `version == "v1"`; they do not re-derive mission semantics.

**Guarantees:** `overall_outcome` remains the coordinator’s aggregate status string (`success`, `failure`, `error` as today). `summary` is always a dict with `message` and `error` fields (error may be null).

## Inspection Contract

Stable inspect returns include `"version": "v1"`:

- **`inspect_team_named_mission_spec`:** `ok`, `version`, `mode`, `overall_outcome`, `step_count`, `preflight`, `preview`, `summary`.
- **`inspect_team_named_mission_spec_checked`:** `ok`, `version`, `mode`, `overall_outcome`, `inspection`, `validation`, `summary`.
- **`inspect_team_named_mission_specs`:** `ok`, `version`, `overall_outcome`, `total_specs`, `ok_count`, `error_count`, `results`, `summary`.
- **`inspect_team_named_mission_specs_checked`:** `ok`, `version`, `overall_outcome`, `inspection`, `validation`, `summary`.

**Guarantees:** Preflight/preview nesting and messages are unchanged. Internal consistency is still checked by `validate_team_named_mission_inspection` and `validate_team_named_mission_specs_inspection` (unchanged required keys; extra `version` on payloads is allowed).

## Result Summary Contract

**`summarize_team_named_mission_result`** (per spec run): compact dict with `version`, `ok`, `mode`, `overall_outcome`, `mission_state`, step counts, index lists, `first_failed_step_index`, `last_step_index_run`, `message`. Sequence vs parallel semantics (e.g. `stopped_early` / `continue_on_failure` null for parallel) are unchanged.

**`summarize_team_named_mission_specs_result`** (batch run): same idea with `total_specs`, `specs_run`, `failed_spec_indices`, `succeeded_spec_indices`, etc., and `version`.

**Semantics:** `mission_state` values such as `completed`, `failed_fast`, `partial_failure`, `invalid` retain their existing meanings. Summaries are pure functions of run/batch dicts.

## Validation Layer

| Function | Role |
|----------|------|
| `validate_team_named_mission_spec_contract` | Single spec v1 shape/version/mode/steps/options/parallel uniqueness. |
| `validate_team_named_mission_specs_contract` | List of specs; aggregates errors with `spec[i]: …` prefixes. |
| `validate_team_named_mission_execution_contract` | Single execute / execute_checked / summarize result shape. |
| `validate_team_named_mission_specs_execution_contract` | Batch execute / checked / batch summarize shape. |
| `validate_team_named_mission_api_manifest` | Manifest consistency (existing). |

All are pure Python, no ROS, no schema frameworks.

## Versioning

- Contract identifier: **`v1`** (`spec["version"]` and payload `"version"` on stable Layer B returns).
- API manifest continues to use `api_version: "v1"` for entrypoint discovery; mission contract `version` is the spec/payload document tag.

## Non-Goals

- No new execution modes, action types, or planner logic.
- No changes to ROS nodes, bridge, or navigation semantics.
- Contract validators do not replace runtime validation inside `normalize_team_named_mission_spec` / preflight; they formalize the v1 document shape for tooling and tests.
- Legacy specs without `version` may still run until callers adopt v1.

## Examples

**Single sequence spec**

```json
{
  "version": "v1",
  "mode": "sequence",
  "steps": [
    {"robot_id": "robot1", "location_name": "base"},
    {"robot_id": "robot2", "location_name": "goal"}
  ],
  "options": {"continue_on_failure": false}
}
```

**Parallel spec**

```json
{
  "version": "v1",
  "mode": "parallel",
  "steps": [
    {"robot_id": "robot1", "location_name": "a"},
    {"robot_id": "robot2", "location_name": "b"}
  ],
  "options": {"max_workers": 2}
}
```

**Checked inspect result (illustrative keys)**

```json
{
  "ok": true,
  "version": "v1",
  "mode": "sequence",
  "overall_outcome": "success",
  "inspection": { "ok": true, "version": "v1", "mode": "sequence", "overall_outcome": "success", "step_count": 1, "preflight": {}, "preview": {}, "summary": {"message": "...", "error": null} },
  "validation": { "ok": true, "overall_outcome": "success", "message": "...", "errors": [] },
  "summary": { "message": "Mission spec inspected and validated successfully.", "error": null }
}
```

**Checked execute result (illustrative keys)**

```json
{
  "ok": true,
  "version": "v1",
  "mode": "sequence",
  "overall_outcome": "success",
  "execution": { "ok": true, "version": "v1", "mode": "sequence", "overall_outcome": "success", "run_result": {}, "execution_summary": {}, "summary": {"message": "...", "error": null} },
  "validation": { "ok": true, "overall_outcome": "success", "message": "...", "errors": [] },
  "summary": { "message": "Mission spec executed and validated successfully.", "error": null }
}
```
