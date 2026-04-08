# Mission Control V4.4 — Bounded real-LLM end-to-end launch smoke (checkpoint)

## Strategic question

Can the **V4.3** launched graph (semantic handoff + mission bridge on one transport topic) run the **real** OpenAI adapter path (`use_deterministic_fake_adapter: false`) while keeping the same envelope, explicit clocks, deterministic gates, frozen transport JSON, and **advisory** blocked bridge outcome?

## Purpose

Add **model-side** realism to the **full E2E launch** without new authority, APIs, schemas, or bridge semantics.

## Audit

- **Topology:** `semantic_handoff_mission_bridge_v43.launch.py` is **sufficient**—no launch or node code changes required. Real vs fake is selected only by the handoff JSON field.
- **Operational:** `OPENAI_API_KEY` must be present in the **handoff node process** environment for a provider success path; optional `OPENAI_BASE_URL`.
- **Blockers:** None in code.

## Already proven

- V4.3 E2E with fake adapter; V4.0 real-adapter bounded failure on V3.9 launch.

## Newly proven by V4.4

- **Automated (CI-friendly):** V4.3 launch subprocess with OpenAI env **stripped** → handoff returns `adapter_failure` / missing key → `NavigateToNamedLocation(base)` is **not** advisory-blocked.
- **Opt-in:** With `RUN_V4_4_LLM_SMOKE=1` and key exported to the **launch** environment, pytest can observe `ingest_stored` + bridge blocked outcome (subject to network and model obeying strict JSON).

## Intentionally not proven

- Guaranteed model compliance, provider bake-off, Nav2 execution, second fact type, product readiness.

## Prerequisites (opt-in success)

```bash
export OPENAI_API_KEY="sk-..."
# optional: export OPENAI_BASE_URL="https://..."
```

Start from a shell that exports these before `ros2 launch` so the handoff child process inherits them.

## Build / launch

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ws
colcon build --packages-select multi_robot_mission_stack_interfaces multi_robot_mission_stack
source install/setup.bash
ros2 launch multi_robot_mission_stack semantic_handoff_mission_bridge_v43.launch.py
```

Use a **wall-aligned** envelope with `"use_deterministic_fake_adapter": false`, then call `/navigate_to_named_location` for `robot1` / `base`.

## Automated tests

**Default (no key, no network):**

```bash
cd /path/to/HDNS
source /opt/ros/humble/setup.bash
source /path/to/ws/install/setup.bash
python3 -m pytest tests/test_semantic_handoff_bridge_real_llm_v44_ros.py::test_v44_e2e_real_adapter_missing_key_handoff_bridge_not_blocked -v
```

**Opt-in E2E real model:**

```bash
export RUN_V4_4_LLM_SMOKE=1
export OPENAI_API_KEY="sk-..."
python3 -m pytest tests/test_semantic_handoff_bridge_real_llm_v44_ros.py::test_v44_e2e_real_llm_handoff_mirror_blocks_bridge -v
```

## Recorded evidence (automated, no provider key)

On the **V4.3** launch graph with OpenAI env stripped from the launch subprocess:

1. Handoff with `use_deterministic_fake_adapter: false` returns `outcome=adapter_failure`, `adapter_reason=adapter_provider_error`, detail containing missing API key text.
2. Immediate `NavigateToNamedLocation(robot1, base)` is **not** `navigation target blocked by peer belief` (no successful ingest / mirror).

Verified by `tests/test_semantic_handoff_bridge_real_llm_v44_ros.py::test_v44_e2e_real_adapter_missing_key_handoff_bridge_not_blocked`.

## Related

- [mission_control_v4_3_checkpoint.md](mission_control_v4_3_checkpoint.md)
- [mission_control_v4_0_checkpoint.md](mission_control_v4_0_checkpoint.md)
