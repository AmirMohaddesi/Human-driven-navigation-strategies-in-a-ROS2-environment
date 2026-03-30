# Step-by-step mission demo script

This is a **minimal, text-only demo wrapper** over the existing mission platform surfaces. It does **not** change the Layer B contract, does **not** modify ROS behavior, and does **not** add planner intelligence.

The script walks through a tiny mission in clear phases and prints compact JSON blobs.

## What the script does

- **Single-spec dry-run**: planner intent → v1 mission spec → `inspect_team_named_mission_spec_checked`
- **Single-spec live (with ROS)**: same, then `execute_team_named_mission_spec_checked`
- **Batch dry-run**: tiny hardcoded batch → `inspect_team_named_mission_specs_checked`
- **Batch live (with ROS)**: same batch → `execute_team_named_mission_specs_checked`

All examples are intentionally **small**:

- Single spec: one step, `robot1` → `base`
- Batch: two specs, `robot1` → `base` and `robot2` → `test_goal`

## Dry-run usage (default path)

From the repo root:

```bash
python3 scripts/demo_step_by_step_mission.py
```

Behavior:

- Builds a tiny intent via the planner seam helper
- Validates and materializes a v1 mission spec
- Runs `inspect_team_named_mission_spec_checked`
- Prints short phase banners and compact JSON for each phase
- Exits **0** only if all phases succeed

You can also pass `--dry-run` explicitly (equivalent to the default):

```bash
python3 scripts/demo_step_by_step_mission.py --dry-run
```

## Live-run usage (with ROS)

This mode assumes the **ROS2 mission stack is already running** (bridge, Nav2, etc.) and available under the existing coordinator expectations.

From the repo root:

```bash
python3 scripts/demo_step_by_step_mission.py --with-ros
```

Behavior:

- Same build + inspect-checked phases as the dry-run
- Then calls `execute_team_named_mission_spec_checked`
- Prints a final compact execution result JSON
- Exits **0** only if execution succeeds

If ROS is not running, this mode will fail in the same way as the existing execute-checked demos.

## Batch usage

To use the tiny hardcoded batch instead of a single spec, add `--batch`.

### Batch dry-run

```bash
python3 scripts/demo_step_by_step_mission.py --batch
python3 scripts/demo_step_by_step_mission.py --batch --dry-run  # equivalent
```

Behavior:

- Uses a small list of two v1 specs
- Calls `inspect_team_named_mission_specs_checked`
- Prints an `== Inspect batch ==` banner and a compact JSON result
- Exits **0** only if the batch inspection succeeds

### Batch live (with ROS)

```bash
python3 scripts/demo_step_by_step_mission.py --batch --with-ros
```

Behavior:

- Runs the checked batch inspect first
- If that passes, calls `execute_team_named_mission_specs_checked`
- Prints `== Inspect batch ==` and `== Execute batch ==` banners and compact JSON
- Exits **0** only if the checked batch execution succeeds

## Pause mode usage

To step through the demo interactively, use `--pause`:

```bash
python3 scripts/demo_step_by_step_mission.py --pause
python3 scripts/demo_step_by_step_mission.py --with-ros --pause
python3 scripts/demo_step_by_step_mission.py --batch --pause
```

After each phase (build, inspect, execute, batch inspect/execute), the script prints:

```text
Press Enter to continue...
```

and waits for you to press Enter. In non-interactive environments the prompt is skipped.

## Notes and non-goals

- This is a **minimal demo wrapper**, not a GUI or dashboard.
- It is designed to be **small, readable, and reproducible**, not feature-complete.
- Live execution paths do **not** start or manage ROS; they assume the existing stack is already up, just like the other Layer B ROS demos.

