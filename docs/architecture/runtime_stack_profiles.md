# Runtime stack profiles (`runtime_stack.launch.py`)

Composable bringup for the TurtleBot3 house simulation and mission stack. Use **`profile`** for opinionated defaults, then override layers with **`enable_*:=true|false`** (or `auto` to follow the profile).

**Semantics:** Profiles only change **which processes start**. They do not change advisory meaning, R1 rules, or mission authority.

## Entry points

| Command | Role |
|---------|------|
| `ros2 launch multi_robot_mission_stack runtime_stack.launch.py` | Primary composable launcher (`profile` defaults to `sim_nav_bridge`). |
| `ros2 launch multi_robot_mission_stack fully_integrated_swarm.launch.py` | **Alias** that includes `runtime_stack` with `profile:=full_demo` (legacy demos and cleanup scripts). |

When you run `ros2 launch … fully_integrated_swarm.launch.py --show-args`, ROS may still print the **inner** default for `profile` (`sim_nav_bridge`); the included launch **forces** `profile:=full_demo` unless you override `profile:=…` on the command line.

## Profiles

| Profile | Gazebo + robots | Nav2 | SLAM | Per-robot RViz | merge_map | merge RViz | mission_bridge | P3 transport params on bridge | P3.2 board | R1 seam | R2 marker | R3 local hold |
|---------|-----------------|------|------|----------------|-----------|------------|----------------|-------------------------------|------------|---------|-----------|---------------|
| `sim_core` | yes | no | no | no | no | no | no | no | no | no | no | no |
| `sim_nav` | yes | yes | no | no | no | no | no | no | no | no | no | no |
| `sim_nav_slam` | yes | yes | yes | no | no | no | no | no | no | no | no | no |
| `sim_nav_bridge` | yes | yes | yes | no | no | no | yes | no | no | no | no | no |
| `sim_nav_bridge_r1` | yes | yes | yes | no | no | no | yes | yes | yes | yes | yes | yes |
| `debug_visibility` | no | no | no | no | no | no | yes (headless) | yes | yes | no | no | no |
| `full_demo` | yes | yes | yes | yes | yes | yes | yes | no | no | no | no | no |

- **P3 transport params on bridge:** `advisory_blocked_passage_*` / `advisory_degraded_passage_*` topic wiring (same strings as historical P3 launches). The bridge still only **ingests blocked** per existing node logic; degraded remains visibility-oriented on the topic.
- **`debug_visibility`:** No simulation — only `mission_bridge_node` (waits for Nav2 disabled) + P3.2 board. For isolated transport/board checks without Gazebo.
- **`sim_nav_bridge_r1`:** **Single** `mission_bridge_node`, P3.2 board, R1 TF observer, bounded R2 marker, and bounded R3 local hold node — avoids duplicate bridge when you want R1/R2/R3 in one process.

## Overrides

All `enable_*` arguments accept `true`, `false`, or `auto` (default `auto`: use the table above).

- `enable_nav`, `enable_slam`, `enable_per_robot_rviz`, `enable_merge_map`, `enable_merge_rviz`, `enable_mission_bridge`, `enable_bridge_advisory_transport`, `enable_p3_board`, `enable_r1_seam`, `enable_r2_visual_consequence`, `enable_r3_local_hold`
- `robot_count` — spawn `robot1` … `robotN` (default `2`). `mission_bridge_node` still maps **robot1 / robot2** in code; larger counts are for spawn/TF scalability experiments only until bridge support extends.

## Combining with R1 overlay (two terminals)

1. **Stack with bridge, no duplicate:** e.g. `profile:=full_demo` (or `sim_nav_bridge`) in terminal A.
2. **R1 + board only** in terminal B: `ros2 launch multi_robot_mission_stack r1_tf_spatial_advisory_visibility.launch.py`  
   This uses **`launch_bridge:=false`** on the P3.2 include so only the board and R1 seam start.

If you need **blocked** advisory **ingest** on the same bridge as the live sim, enable transport on that bridge, e.g.:

`ros2 launch multi_robot_mission_stack runtime_stack.launch.py profile:=full_demo enable_bridge_advisory_transport:=true`

(or use **`profile:=sim_nav_bridge_r1`** for one-terminal R1 + advisory transport + board.)

## Out of scope

- Fleet-scale advisory logic, new seams, or planner/costmap fusion.
- Extending `mission_bridge_node` robot ID map for `robot_count` &gt; 2 (spawn/Nav layers scale; bridge contract does not yet).
