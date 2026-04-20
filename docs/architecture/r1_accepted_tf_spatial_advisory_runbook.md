# R1 Accepted Seam Runbook (TF spatial -> degraded advisory, with optional R2 marker + R3 local hold)

## What R1 is

R1 is an accepted, bounded runtime proof seam:

- **single runtime source family:** robot-to-robot TF spatial relationship
- **single trigger:** distance between robot base frames is at or below threshold
- **single output:** advisory-only `degraded_passage` JSON on `/semantic/degraded_passage_p1_1`
- **visibility path:** existing P3.2 board

No mission/control authority is added.

## Trigger and output contract

- Observed source topics:
  - `/robot1_ns/tf`
  - `/robot2_ns/tf`
- Trigger condition:
  - Euclidean distance(`base_footprint` pose from robot1 TF stream, robot2 TF stream) `<= distance_threshold_m`
- Output topic:
  - `/semantic/degraded_passage_p1_1`
- Output proof marker in payload:
  - `location_ref='tf:/robot1_ns/tf<->/robot2_ns/tf:dist<=2.50'` (threshold reflects launch params)

## Clean proof run (no manual degraded injection)

Terminal 1 (sim + bridge — same as historical ``fully_integrated``):

```bash
cd /home/amix/HDNS
bash scripts/cleanup_baseline_runtime.sh pre
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch multi_robot_mission_stack fully_integrated_swarm.launch.py
```

**Single-terminal alternative (one bridge + board + R1 + R2 marker, no duplicate bridge):**

```bash
ros2 launch multi_robot_mission_stack runtime_stack.launch.py profile:=sim_nav_bridge_r1
```

Terminal 2 (R1 seam + P3.2 board only — does **not** start a second ``mission_bridge_node``):

```bash
cd /home/amix/HDNS
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch multi_robot_mission_stack r1_tf_spatial_advisory_visibility.launch.py
```

See [runtime_stack_profiles.md](runtime_stack_profiles.md) for composable profiles.

Optional Terminal 3 (capture one raw degraded advisory):

```bash
cd /home/amix/HDNS
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /semantic/degraded_passage_p1_1 --once
```

## Proof markers to look for

In Terminal 2:

- Seam startup:
  - `R1 TF seam active: ...`
- Seam emission:
  - `R1 TF seam emitted degraded advisory (distance=... location_ref=... belief_id=...)`
- Board ingest:
  - `[P3.2_ROW] lane=degraded ... status=VALID ... location_ref='tf:/robot1_ns/tf<->/robot2_ns/tf:dist<=...'`
- R2 visible consequence marker:
  - `R2 visual marker emitted from R1 degraded advisory (belief_id=... ttl=...)`
  - RViz marker topic: `/semantic/r2/degraded_marker` (configured in `rviz/merge_map.rviz`)
- R3 local behavior consequence:
  - `R3 hold armed from R1 degraded advisory (...)`
  - `R3 hold released (...)`
  - target cmd_vel topic (default): `/robot1_ns/cmd_vel`

## What R1 does not authorize

- No mission planner/coordinator authority
- No navigation command issuance from seam
- No blocked runtime generation
- No multi-source fusion (planner/costmap/map/sensor)
- No product/dashboard/platform expansion

