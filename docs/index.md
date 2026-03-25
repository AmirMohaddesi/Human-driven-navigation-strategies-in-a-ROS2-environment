# ROS2 Multi-Agent Disaster Response Platform

Welcome to the official documentation for the **disaster_response_swarm** package — 
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

ros2 launch disaster_response_swarm fully_integrated_swarm.launch.py
```

## 📚 Documentation Sections

- [Installation](installation.md)  
- [Architecture Overview](architecture.md)  
- [Launch Files Guide](launch_files.md)  
- [Parameter Files Explained](config_parameters.md)  
- [Map Merging & YOLO Detection](advanced_features.md)  
- [Troubleshooting](troubleshooting.md)  
- [Contributing](contributing.md)  
