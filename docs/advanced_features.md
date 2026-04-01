# Map Merging & YOLO Detection

## Map Merging
The `merge_map_node.py` combines individual SLAM maps (from each robot’s **`local_map`**) into **`/merged_map`**. That output is **auxiliary** for this stack; per-robot Nav2 uses namespaced **`map_server`** + **`/{robot}_ns/map`** (see `docs/architecture/mission_system_runbook.md` §J).

## YOLO Detection
`yolo_detection_node.py` provides an (experimental) YOLOv3-based detector.

YOLOv3 weights/config are not included in this repository; provide `weights_path` and `cfg_path` ROS parameters when launching/running the node.
