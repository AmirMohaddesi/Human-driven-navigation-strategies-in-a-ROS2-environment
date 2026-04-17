# P3.5 — P3 live demo quickstart (visibility-only)

## Prereqs

- ROS 2 workspace built: `colcon build --symlink-install` from the repo root.
- Shell: `source install/setup.bash` in every terminal below.

## Demo surfaces now (bounded)

- **Integrated simulation surface**
  - Command: `ros2 launch multi_robot_mission_stack fully_integrated_swarm.launch.py`
  - Proves: project-native Gazebo + multi-robot runtime stack bringup.
  - Does not prove: mission authorization or seam-origin advisory by itself.

- **P3 manual advisory visibility surface**
  - Commands:
    - `ros2 launch multi_robot_mission_stack p3_2_dual_advisory_visibility.launch.py`
    - `./scripts/p3_3_operator_p3_2_visibility_demo.sh`
  - Proves: visibility lane behavior for valid blocked/degraded payloads and malformed blocked payload resilience.
  - Does not prove: runtime-grounded seam origin (uses manual publishes).

- **R1 runtime-grounded advisory surface**
  - Commands:
    - `ros2 launch multi_robot_mission_stack fully_integrated_swarm.launch.py`
    - `ros2 launch multi_robot_mission_stack r1_tf_spatial_advisory_visibility.launch.py`
  - Proves: accepted R1 TF-distance seam emits degraded advisories into existing P3 degraded lane.
  - Proof markers:
    - seam log: `R1 TF seam emitted degraded advisory`
    - board marker: `location_ref='tf:/robot1_ns/tf<->/robot2_ns/tf:dist<=...'`
  - Does not prove: control authority, coordinator actioning, or multi-source fusion.

## 3-7 minute live walkthrough (presenter script)

Use this sequence to demo the current branch quickly with clear boundaries.

### Phase 1 — Integrated simulation

Commands (Terminal A):

```bash
cd /home/amix/HDNS
bash scripts/cleanup_baseline_runtime.sh pre
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch multi_robot_mission_stack fully_integrated_swarm.launch.py
```

Audience should notice:

- Gazebo opens with house world and two robots.
- Runtime stack processes come up (Nav2/SLAM/RViz surfaces).

Presenter narration (say 1-3 lines):

- "This is the project-native integrated bringup, not a toy launch."
- "We can see robots and runtime processes are alive before any advisory demo."

Boundary / non-claim:

- This phase proves integrated runtime bringup only; it does not prove advisory seam origin or control authority.

### Phase 2 — P3 manual advisory visibility demo

Commands (Terminal B, then C):

```bash
cd /home/amix/HDNS
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch multi_robot_mission_stack p3_2_dual_advisory_visibility.launch.py
```

```bash
cd /home/amix/HDNS
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/p3_3_operator_p3_2_visibility_demo.sh
```

Audience should notice:

- `[P3.2_ROW] lane=blocked ... status=VALID` after blocked step.
- `[P3.2_ROW] lane=degraded ... status=VALID` after degraded step.
- `[P3.2_EVT] ... status=UNPARSEABLE_JSON` after malformed step, with board still alive.

Presenter narration (say 1-3 lines):

- "This path shows visibility behavior and resilience on advisory transport."
- "Valid payloads are surfaced, malformed payloads are contained, and observers stay up."

Boundary / non-claim:

- This phase uses manual publishes; it proves visibility and robustness, not runtime-grounded seam origin or mission authority.

### Phase 3 — R1 runtime-grounded advisory seam

Commands (Terminal B, clean run without wrapper):

```bash
cd /home/amix/HDNS
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch multi_robot_mission_stack r1_tf_spatial_advisory_visibility.launch.py
```

Optional capture (Terminal C):

```bash
cd /home/amix/HDNS
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /semantic/degraded_passage_p1_1 --once
```

Audience should notice:

- Seam log: `R1 TF seam emitted degraded advisory`.
- Board marker: `location_ref='tf:/robot1_ns/tf<->/robot2_ns/tf:dist<=...'`.

Presenter narration (say 1-3 lines):

- "Now the degraded advisory is runtime-grounded from TF robot-to-robot distance."
- "It reuses the same P3 visibility lane, but source is live runtime, not manual publish."

Boundary / non-claim:

- This phase proves a bounded advisory-only seam; it does not authorize planner/coordinator control or multi-source semantic fusion.

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

## R1 accepted seam proof (clean, no manual degraded publish)

Use this when you need to prove seam-origin degraded traffic from runtime TF data only.

Terminal 1:

```bash
cd /home/amix/HDNS
bash scripts/cleanup_baseline_runtime.sh pre
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch multi_robot_mission_stack fully_integrated_swarm.launch.py
```

Terminal 2:

```bash
cd /home/amix/HDNS
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch multi_robot_mission_stack r1_tf_spatial_advisory_visibility.launch.py
```

Proof markers:

- seam log includes `R1 TF seam emitted degraded advisory`
- degraded board row/event includes `location_ref='tf:/robot1_ns/tf<->/robot2_ns/tf:dist<=...'`
- this run intentionally excludes the P3.3 wrapper degraded publish step

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
