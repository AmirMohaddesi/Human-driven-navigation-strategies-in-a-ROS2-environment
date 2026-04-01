# Troubleshooting

## Common Issues

### Gazebo models not found
Ensure `GAZEBO_MODEL_PATH` is set correctly.

### Robots not moving
Check if Nav2 stack is launched and goals are sent to the correct namespace.

### Map not merging / wrong map readiness signal
`merge_map_node` subscribes to each robot’s **`local_map`** (SLAM topic), not namespaced **`/map`**. For **per-robot Nav2** readiness, use **`/{robot}_ns/map`** (`map_server`), lifecycles, and **`/{robot}_ns/tf`** (see **`map` → `odom`**, with **SLAM Toolbox** as the intended source when **`amcl.tf_broadcast`** is false)—not **`/merged_map`** or a generic root **`/map`**. See `docs/architecture/mission_system_runbook.md` §J. Optionally verify `map_merge_params.yaml` if you care about merge visualization only.
