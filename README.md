# ROS 2 Multi-Agent Disaster Response Platform

A ROS 2 (Humble) multi-robot simulation platform for disaster response scenarios, built around:
- Gazebo (multi-robot TurtleBot3 in isolated namespaces)
- SLAM Toolbox (per-robot mapping/localization)
- Navigation2 (Nav2) (per-robot autonomy)
- Map merging (publishes a merged occupancy grid for multi-robot analysis)

Demo video: [YouTube](https://www.youtube.com/watch?v=nfTs7sWDnww)

## System Overview
Two TurtleBot3 robots are spawned in Gazebo (2 robots: `robot1_ns` and `robot2_ns`) and run **SLAM Toolbox** plus **Nav2** independently in their own ROS namespaces.
Each robot builds a local occupancy grid while Nav2 plans locally on its map.
A dedicated `merge_map_node` coordinates multi-robot understanding by fusing the per-robot maps into a shared `/merged_map` for global visualization/analysis.
“Human-driven navigation strategies” in this codebase means strategy-like landmark/path selection from **map-derived free space** and validating candidate routes using Nav2 path planning.
The result is a realistic robotics workflow: decentralized autonomy + centralized coordination for shared situational awareness.

## Architecture (quick diagram)

```text
                          Gazebo world
                               |
                               v
          +--------------------+--------------------+
          |                                         |
       robot1_ns                                robot2_ns
          |                                         |
          |   SLAM Toolbox                          |   SLAM Toolbox
          |   publishes local maps                  |   publishes local maps
          |   (/robot1_ns/local_map)                |   (/robot2_ns/local_map)
          |                                         |
          |   Nav2 (navigation autonomy)            |   Nav2 (navigation autonomy)
          |   publishes /robot1_ns/map              |   publishes /robot2_ns/map
          |   and /robot1_ns/amcl_pose              |   and /robot2_ns/amcl_pose
          |                                         |
          |   graph_construction_node               |   graph_construction_node
          |   - reads /robot1_ns/map                |   - reads /robot2_ns/map
          |   - reads /robot1_ns/amcl_pose          |   - reads /robot2_ns/amcl_pose
          |   - calls Nav2 ComputePathToPose        |   - calls Nav2 ComputePathToPose
          |   - publishes /robot1_ns/survey_markers
          +--------------------+--------------------+
                               |
                               v
                      merge_map_node
                 (fuses local maps -> /merged_map)
                               |
                               v
                              RViz
                    (visualizes /merged_map)
```

Each robot runs SLAM Toolbox + Nav2 in its own namespace (decentralized autonomy), while `merge_map_node` centrally fuses the SLAM-produced local maps into `/merged_map` for shared visualization. `graph_construction_node` exists per namespace and uses the robot’s map + AMCL pose to request feasible paths from Nav2, publishing `/<robot_namespace>/survey_markers` for inspection.

## Key Features
- Multi-robot simulation in Gazebo with per-robot namespaces.
- Per-robot SLAM + Nav2 bringup.
- `merge_map_node`: merges per-robot local maps into a single occupancy grid topic: `/merged_map`.
- `graph_construction_node`: a unified path-planning test node that publishes visualization markers on `/<robot_namespace>/survey_markers` after calling Nav2’s `ComputePathToPose`.

## Requirements
- Ubuntu 22.04 + ROS 2 Humble (or ROS 2 Humble on WSL2 with Gazebo support)
- Gazebo + ROS 2 integration packages
- TurtleBot3 simulation packages
- Nav2 bringup packages

> This repository is meant to be used in simulation. ROS 2 packages like `nav2_bringup`, `slam_toolbox`, and `turtlebot3_gazebo` must be installed from apt.

## How to run in 2 minutes
Assuming you already built the workspace once with the Quickstart below, run:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash

export TURTLEBOT3_MODEL=waffle
ros2 launch disaster_response_swarm fully_integrated_swarm.launch.py
```

Expected result (what you’ll see):
- Gazebo starts and spawns **two robots** (`robot1_ns`, `robot2_ns`).
- RViz starts and shows navigation/SLAM visualization; `merge_map_node` publishes a merged view on `/merged_map`.
- `graph_construction_node` publishes marker arrays you can inspect at:
  - `/robot1_ns/survey_markers`
  - `/robot2_ns/survey_markers`

## Quickstart (Build + Run)

### 1) Install ROS 2 and dependencies (apt)
```bash
sudo apt update
sudo apt install ros-humble-desktop \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-gazebo-* \
  ros-humble-turtlebot3* \
  ros-humble-rviz2
```

### 2) Build this package with colcon
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/AmirMohaddesi/Human-driven-navigation-strategies-in-a-ROS2-environment.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3) Run the full integrated multi-robot simulation
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch disaster_response_swarm fully_integrated_swarm.launch.py
```

## Optional: YOLO Vision Detection Node
The repo includes a `yolo_detection_node`, but **YOLOv3 weights/config are not shipped in this repository**.

To run it, download the YOLOv3 weights + cfg files yourself, then point the node at them:
```bash
ros2 run disaster_response_swarm yolo_detection_node \
  --ros-args \
  -p weights_path:=/path/to/yolov3.weights \
  -p cfg_path:=/path/to/yolov3.cfg \
  -p display:=false
```

## What makes this interesting
- Multi-robot coordination is explicit: **decentralized** SLAM+Nav2 runs per robot namespace, while a **centralized** `merge_map_node` produces `/merged_map` for shared situational awareness.
- The “strategy” layer is map-driven and robotics-realistic: `graph_construction_node` treats **free-space landmark candidates** as human-like waypoints, then verifies feasibility by calling Nav2’s `ComputePathToPose` rather than “guessing” motion.
- This makes it easier to evaluate human-inspired navigation ideas in simulation with real-time TF, costmaps, and localization feedback.

## Limitations (honest)
- Simulation-first: launches target Gazebo workflows and the provided RViz configs.
- YOLO is optional/experimental: detections are disabled unless you provide YOLOv3 weights + cfg via ROS parameters.
- Map merging is primarily for shared visualization/analysis; the default Nav2 bringup still consumes the map YAML passed to Nav2 for each robot (merged map is not a fully end-to-end planning input).

## Documentation
In-repo docs (architecture, launch files, parameters, troubleshooting):
- `docs/index.md`
- `docs/installation.md`
- `docs/architecture.md`
- `docs/launch_files.md`

## License
Apache License 2.0. See `LICENSE`.

