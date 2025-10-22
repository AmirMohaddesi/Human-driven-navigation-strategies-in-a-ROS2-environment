# ROS2 Multi-Agent Disaster Response Platform 🚨🤖

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)
![License](https://img.shields.io/badge/License-MIT-green)

<p align="center">
  <a href="https://www.youtube.com/watch?v=nfTs7sWDnww" target="_blank">
    <img src="https://img.youtube.com/vi/nfTs7sWDnww/maxresdefault.jpg"
         alt="Watch ROS2 Multi-Agent Disaster Response Platform Demo" width="720" style="border-radius:12px;">
  </a>
</p>


A **ROS 2 (Humble Hawksbill)** multi-robot simulation platform for disaster response scenarios.  
Deploy multiple [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) robots in **Gazebo** to perform:

- Simultaneous Localization and Mapping ([SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox))
- Autonomous Navigation ([Navigation2](https://docs.nav2.org/))
- Coordinated multi-agent exploration with **map merging**
- YOLO-based vision detection for object recognition in disaster zones

---

## 📖 Table of Contents
- [Features](#features)
- [Installation](#installation)
- [Running the Simulation](#running-the-simulation)
- [Visualization](#visualization)
- [Customization](#customization)
- [Upcoming Features](#upcoming-features)
- [Contributing](#contributing)
- [License](#license)

---

## ✨ Features
| Capability                     | Description |
|--------------------------------|-------------|
| **Multi-Robot Simulation**     | Spawn multiple TurtleBot3 agents in Gazebo with isolated namespaces |
| **SLAM Toolbox**               | Real-time mapping from LIDAR data |
| **Navigation2 (Nav2)**         | Autonomous path planning and obstacle avoidance |
| **Map Merging**                | Combine maps from multiple robots into a single global map |
| **YOLO Detection**             | Vision-based object detection for identifying hazards or victims |
| **Visualization**              | TF trees, robot models, maps, and sensor data in RViz |

---
## Documentation

This pacckage full documentation can be found at ([here](https://amirmohaddesi.github.io/Human-driven-navigation-strategies-in-a-ROS2-environment/))

## Installation

**Tested on:** Ubuntu 22.04 + ROS 2 Humble

### 1️⃣ Install ROS 2 Humble
Follow the [official ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html)  
Or run:

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

### 2️⃣ Install dependencies
```bash
sudo apt install   ros-humble-navigation2   ros-humble-nav2-bringup   ros-humble-slam-toolbox   ros-humble-gazebo-*   ros-humble-turtlebot3*   ros-humble-rviz2
```

### 3️⃣ Environment configuration
Add to your `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

---

## Running the Simulation

### 1️⃣ Clone and build
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/AmirMohaddesi/disaster_response_swarm.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2️⃣ Launch the fully integrated swarm
```bash
ros2 launch disaster_response_swarm fully_integrated_swarm.launch.py
```
This will:
- Spawn multiple robots in **Gazebo**
- Start SLAM + Nav2 for each robot
- Enable **map merging** to combine their maps
- Assign unique namespaces (`/tb3_0`, `/tb3_1`, …)

---

## Visualization

Launch RViz with the provided multi-robot config:
```bash
rviz2 -d src/disaster_response_swarm/rviz/merge_map.rviz
```
**Suggested displays**:
- **TF**: View transforms per robot namespace
- **RobotModel**: Show 3D models (`/tb3_X/robot_description`)
- **LaserScan**: View LIDAR scans (`/tb3_X/scan`)
- **Map**: Global map from merged SLAM data (`/map`)

---

## Customization

| What to change              | How to do it |
|-----------------------------|--------------|
| **Number of robots**        | Edit `fully_integrated_swarm.launch.py` robot list |
| **Start positions**         | Modify pose parameters in launch file |
| **SLAM/Nav2 parameters**    | Adjust YAML files in `config/` |
| **Robot model**             | Change `TURTLEBOT3_MODEL` to `burger` or `waffle` |
| **Vision detection**        | Configure `disaster_response_swarm/vision/yolo_detection_node.py` |

---

## Upcoming Features
- Enhanced disaster world environments (`worlds/disaster_world.world`)
- GUI for easier configuration
- Support for heterogeneous robot teams (ground + aerial)
- Improved victim detection with multimodal sensors

---

## Contributing
We welcome:
- Feature suggestions
- Bug reports
- Pull requests

Please open an [issue](https://github.com/AmirMohaddesi/disaster_response_swarm/issues) before major changes.

---

## License
This project is licensed under the [MIT License](LICENSE).

---

**Happy exploring — and may your robots always find the safest path!**
