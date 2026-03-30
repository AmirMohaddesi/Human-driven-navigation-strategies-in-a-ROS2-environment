# ROS2 Multi-Agent Disaster Response Platform

Welcome to the official documentation for the **multi_robot_mission_stack** package —
a ROS 2 (Humble) multi‑robot platform for disaster response simulation using TurtleBot3.

This site provides installation guides, architecture overviews, and advanced usage tips 
to help you get the most out of the platform.

## 🚀 Quick Start

```bash
sudo apt update
sudo apt install ros-humble-desktop \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-gazebo-* \
  ros-humble-turtlebot3* \
  ros-humble-rviz2

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/AmirMohaddesi/Human-driven-navigation-strategies-in-a-ROS2-environment.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch multi_robot_mission_stack fully_integrated_swarm.launch.py
```

## 📋 Manager handoff (Layer B checkpoint)

- [Capability snapshot](handoff/capability_snapshot.md) · [Golden-path demo](handoff/golden_path_demo.md) · [Non-goals / deferred](handoff/non_goals_deferred.md) · [Handoff summary](handoff/manager_handoff_summary.md)

## 📚 Documentation Sections

- [Installation](installation.md)  
- [Architecture Overview](architecture.md)  
- [Launch Files Guide](launch_files.md)  
- [Parameter Files Explained](config_parameters.md)  
- [Map Merging & YOLO Detection](advanced_features.md)  
- [Troubleshooting](troubleshooting.md)  
- [Contributing](contributing.md)  
