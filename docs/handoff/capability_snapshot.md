# Layer B capability snapshot (manager reporting)

**Purpose:** One-page view of what the stack can do today at the team / mission-spec boundary.  
**Scope:** `multi_robot_mission_stack` coordinator (Layer B) over the validated ROS2 mission bridge (Layer A).

## Execution reality

| Area | Status | Notes |
|------|--------|--------|
| Single-robot named navigation via ROS2 | **Supported** | Facade + wait utilities; per-call lifecycle |
| Team-level sequence / parallel named missions | **Supported** | Thin coordinator helpers over Layer A |
| Mission specs (single + batch) | **Supported** | Normalize, preflight, preview, inspect, run, execute, summaries + validators |
| Checked seams (inspect + execute + batch) | **Supported** | Combines primary result with invariant validation payloads |
| API manifest + request validation + plan object | **Supported** | Static manifest, resolver, `validate_team_named_mission_api_request`, `plan_team_named_mission_api_call` |

## Stable Layer B entrypoints (v1 contract)

**Single spec:** `inspect_team_named_mission_spec`, `inspect_team_named_mission_spec_checked`, `execute_team_named_mission_spec`, `execute_team_named_mission_spec_checked`.

**Batch specs:** `inspect_team_named_mission_specs`, `inspect_team_named_mission_specs_checked`, `execute_team_named_mission_specs`, `execute_team_named_mission_specs_checked`.

**Contract helpers:** `get_team_named_mission_api_manifest`, `validate_team_named_mission_api_manifest`, `resolve_team_named_mission_api_entrypoint`, `validate_team_named_mission_api_request`, `plan_team_named_mission_api_call`.

## Quick verification (no ROS)

- `python3 -m pytest tests/test_coordinator.py -q`
- `python3 scripts/validate_coordinator_api_manifest.py`
- `python3 scripts/validate_coordinator_inspect_checked.py`
- `python3 scripts/validate_coordinator_batch_inspect_checked.py`

## Quick verification (live stack)

- `python3 scripts/validate_coordinator_execute_checked_ros.py`
- `python3 scripts/validate_coordinator_execute_batch_checked_ros.py`

## Reporting one-liner

The platform delivers **trusted, structured team mission-spec workflows** (offline inspection and optional live execution) with **explicit validation and a versioned API manifest** for higher layers to bind against.
