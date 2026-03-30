# Planner seam demo (Layer C)

Minimal integration above the frozen Layer B contract: structured **intent** becomes a **v1 mission spec**, passes **contract validation**, then **`inspect_team_named_mission_spec_checked`** and **`execute_team_named_mission_spec_checked`** (in order; execution only if inspection succeeds).

## Runnable artifact

From the repo root:

```bash
python3 scripts/planner_seam_demo_report.py
```

The script prints a short text summary and a JSON report. It uses **real** build, contract check, and **checked inspection** (pure Python, no ROS). **Execution** is a **labeled stub** so the demo runs in CI; for a live stack, call `run_planner_team_named_mission_checked` without overriding `execute_checked`.

## What this is not

No replanning, retries, or Layer B API changes. Pause here before adding planner sophistication.

## Code entrypoints

- `multi_robot_mission_stack.planner_seam` — `build_team_named_mission_spec_v1_from_intent`, `run_planner_team_named_mission_checked`
- Contract reference — `docs/mission_contract_v1.md`
