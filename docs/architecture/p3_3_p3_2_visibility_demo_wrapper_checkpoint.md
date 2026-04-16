# P3.3 — P3.2 visibility-only demo wrapper (checkpoint)

## Strategic question

Can an operator run one short script to drive a standard blocked -> degraded -> malformed advisory
sequence while watching P3.2, without having to remember individual publish commands?

## Answer (this milestone)

Yes. `scripts/p3_3_operator_p3_2_visibility_demo.sh` provides a terminal wrapper that:

- guides Terminal 1 launch of `p3_2_dual_advisory_visibility.launch.py`
- supports `--dry-run` for no-side-effect inspection
- runs a fixed sequence of three publishes:
  1. valid blocked advisory
  2. valid degraded advisory
  3. malformed payload on blocked topic

The wrapper uses only the existing P1.1 helper chain and `ros2 topic pub` on existing advisory
topics.

## Honesty / scope

- Visibility-only wrapper around existing P3.2 + P1.1 helpers.
- No control authority added: no service/action calls, no mission goals, no cancel paths.
- No new semantic meaning introduced.
- `shared_space_caution` remains out of runtime scope for this demo surface.
- No browser UI, backend orchestration, persistence, auth, or observability platform work.

## Usage

Terminal 1:

```bash
cd /path/to/HDNS
source install/setup.bash
ros2 launch multi_robot_mission_stack p3_2_dual_advisory_visibility.launch.py
```

Terminal 2:

```bash
cd /path/to/HDNS
source install/setup.bash
./scripts/p3_3_operator_p3_2_visibility_demo.sh
```

Dry-run (no side effects):

```bash
./scripts/p3_3_operator_p3_2_visibility_demo.sh --dry-run
```

## What to observe on P3.2 board

- After blocked publish: blocked row shows `status=VALID` and blocked `fact_type`.
- After degraded publish: degraded row shows `status=VALID` and degraded `fact_type`.
- After malformed publish: blocked malformed counter increments; recent events include
  `UNPARSEABLE_JSON`; board keeps running.

## Verification

```bash
cd /path/to/HDNS
python3 -m pytest tests/test_p3_3_operator_p3_2_visibility_demo_dry_run.py -q --tb=short
```

## Related

- [p3_4_dual_advisory_visibility_interpretation_cheatsheet.md](p3_4_dual_advisory_visibility_interpretation_cheatsheet.md)
- [p3_2_dual_advisory_visibility_checkpoint.md](p3_2_dual_advisory_visibility_checkpoint.md)
- [p3_1_dual_advisory_visibility_checkpoint.md](p3_1_dual_advisory_visibility_checkpoint.md)
- [p1_1_headless_dual_advisory_bridge_demo_checkpoint.md](p1_1_headless_dual_advisory_bridge_demo_checkpoint.md)
