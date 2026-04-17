# P3.5 — P3 live demo quickstart (visibility-only)

## Prereqs

- ROS 2 workspace built: `colcon build --symlink-install` from the repo root.
- Shell: `source install/setup.bash` in every terminal below.

## Fastest live path (P3.2 + P3.3)

**Terminal 1 — board + bridge**

```bash
cd /path/to/HDNS
source install/setup.bash
ros2 launch multi_robot_mission_stack p3_2_dual_advisory_visibility.launch.py
```

**Terminal 2 — scripted publishes (blocked → degraded → malformed)**

```bash
cd /path/to/HDNS
source install/setup.bash
./scripts/p3_3_operator_p3_2_visibility_demo.sh
```

Press Enter when prompted (after Terminal 1 is running).

**What you should see:** `[P3.2_BOARD]`, `[P3.2_ROW]`, `[P3.2_EVT]` lines on Terminal 1 as messages arrive.

## Demo evidence — successful operator run (checklist)

Use this to confirm the same bounded sequence you expect after a good run.

**Smallest live check (optional, before the full wrapper)** — Terminal 2:

```bash
python3 scripts/p1_1_print_blocked_advisory_wire.py | python3 scripts/p1_1_publish_std_string_once.py /semantic/blocked_passage_p1_1
```

**Terminal 1 — patterns that mean “working”**

- After **valid blocked** traffic: `[P3.2_ROW] lane=blocked` shows `status=VALID`, `fact_type='blocked_passage'`, `valid` counter increases; bridge may log `advisory blocked_passage ingested at bridge`.
- After **valid degraded** traffic: `[P3.2_ROW] lane=degraded` shows `status=VALID`, `fact_type='degraded_passage'`, `valid` counter increases.
- After **malformed** payload on blocked topic: `[P3.2_EVT]` includes `status=UNPARSEABLE_JSON`; blocked row `malformed` counter increases; board keeps printing (no crash).

**What those three steps mean (this demo only)**

- **Blocked / degraded:** valid JSON on the existing P1.1 advisory transport topics — **transport + ingest observation**, not mission authorization.
- **Malformed:** intentionally bad JSON on the blocked topic — proves the **visibility board** survives bad data and surfaces parse failure without claiming new semantics.

**What this does not prove:** coordinator control, Nav2 goals, `shared_space_caution` runtime, or any dashboard/product surface.

## Read-only digest only (P3.1)

```bash
ros2 launch multi_robot_mission_stack p3_1_dual_advisory_visibility.launch.py
```

Expect `[P3.1]` digest lines instead of the board.

## Static interpretation (no ROS)

- [p3_4_dual_advisory_visibility_interpretation_cheatsheet.md](p3_4_dual_advisory_visibility_interpretation_cheatsheet.md)

## Honesty

Visibility-only observers; no control authority. See P3 checkpoint docs for scope.
