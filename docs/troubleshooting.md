# Troubleshooting

## Common Issues

### Gazebo models not found
Ensure `GAZEBO_MODEL_PATH` is set correctly.

### Robots not moving
Check if Nav2 stack is launched and goals are sent to the correct namespace.

### Map not merging
Verify `map_merge_params.yaml` settings and that all robots publish `/map` topics.
