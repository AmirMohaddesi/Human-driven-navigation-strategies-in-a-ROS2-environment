# Architecture Overview

The **disaster_response_swarm** platform consists of several ROS 2 nodes and launch files enabling 
multi-robot SLAM, navigation, and map merging.

## Components

- **SLAM Toolbox** — Handles mapping and localization for each robot.
- **Navigation2 (Nav2)** — Path planning and motion control.
- **Map Merge Node** — Merges local maps into a global map.
- **YOLO Detection Node** — Performs vision-based object detection.
- **Gazebo Simulation** — Simulates robots in a 3D world.

## Node Diagram

```plaintext
[ Gazebo World ] --> [ Robot State Publishers ] --> [ SLAM Nodes ] --> [ Nav2 ]
                                             \--> [ YOLO Detection ]

[ SLAM Nodes ] --> [ Map Merge Node ] --> [ Global Map Topic ]
```
