# Manager handoff — Layer B mission API checkpoint

**Date context:** March 2026 (repository state at handoff).  
**Intent:** Close this phase with a **manager-ready checkpoint**: clear capabilities, a demo path, explicit deferrals, and a single strategic fork.

## Delivered in this phase

1. **Real execution path** — Robots can run named-navigation missions through the validated ROS2 mission layer (Layer A).
2. **Team coordination** — Thin Layer B helpers for single, sequential, and parallel named-navigation missions.
3. **Mission-spec workflows** — Single- and batch-spec **inspect**, **execute**, **summarize**, and **checked** variants that attach validation payloads for higher layers.
4. **Trust surface** — API **manifest** (v1), manifest validator, **request** validator, **resolver**, **plan** object (`plan_team_named_mission_api_call`), and focused **tests / scripts**.

## Artifacts in this folder

| Artifact | File |
|----------|------|
| Capability snapshot (reporting) | [capability_snapshot.md](capability_snapshot.md) |
| Golden-path demo (checked flows) | [golden_path_demo.md](golden_path_demo.md) |
| Non-goals / deferred work | [non_goals_deferred.md](non_goals_deferred.md) |

**Demo driver:** `scripts/demo_layer_b_golden_path.sh` (offline by default; `--with-ros` for execute-checked).

## Quality bar

- `python3 -m pytest tests/test_coordinator.py -q` — coordinator unit coverage for Layer B helpers and validators.
- Offline scripts exit `0` only when the reported `ok` field is true; ROS scripts assume a live stack.

## Recommended next step (choose one branch)

- **Planner integration** above the stable Layer B API (manifest + checked entrypoints + plan object), **or**
- **More realistic distributed execution** below Layer B (ownership, scaling, runtime hardening) **without** breaking the current contract.

Do **not** blend both in one increment; pick one to preserve velocity and clear accountability.

## Sign-off line (optional)

Layer B provides a **versioned, validated mission-spec API** suitable for product and autonomy teams to bind planners or execution infrastructure against, with **checked seams** for operational confidence.
