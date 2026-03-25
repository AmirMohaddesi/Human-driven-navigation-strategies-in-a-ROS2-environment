# Map Merging & YOLO Detection

## Map Merging
The `merge_map_node.py` combines individual SLAM maps into a unified map topic `/map`.

## YOLO Detection
`yolo_detection_node.py` provides an (experimental) YOLOv3-based detector.

YOLOv3 weights/config are not included in this repository; provide `weights_path` and `cfg_path` ROS parameters when launching/running the node.
