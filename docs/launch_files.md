# Launch Files Guide

## fully_integrated_swarm.launch.py
Starts the complete simulation with multiple robots, SLAM, Nav2, and map merging.

```bash
ros2 launch multi_robot_mission_stack fully_integrated_swarm.launch.py
```

**Validation baseline entrypoint:** this is the canonical launch for live mission-layer golden-path validation.
For acceptance procedure and expected validator outputs, use
`docs/architecture/mission_system_runbook.md` → **I. Locked Golden Path Baseline (Live ROS2)**.

**Navigation readiness:** per-robot Nav2 depends on **namespaced** map/localization/TF (**`/{robot}_ns/map`**, SLAM for **`map`→`odom`**); **`/merged_map`** from merge is **not** that gate—see the same runbook → **§J**.

## navigation_launch.py
Brings up the Nav2 stack for a single robot.

## slam_with_sim_camera.launch.py
Runs SLAM with simulated camera sensors.

---
**Tip:** Launch files are located in the `launch/` directory.
