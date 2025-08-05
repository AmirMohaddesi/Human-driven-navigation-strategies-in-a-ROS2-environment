# ROS2 Multi-Agent Disaster Response Platform 🚨🤖

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)
![License](https://img.shields.io/badge/License-MIT-green)

A **ROS 2 (Humble Hawksbill)** multi-robot simulation platform for disaster response scenarios.  
Deploy multiple [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) robots in **Gazebo** to perform:

- Simultaneous Localization and Mapping ([SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox))
- Autonomous Navigation ([Navigation2](https://docs.nav2.org/))
- Coordinated multi-agent exploration

Designed to be **accessible to researchers**, even with minimal ROS2 or Ubuntu experience.

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
| **Visualization**              | TF trees, robot models, maps, and sensor data in RViz |

---

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
📚 References:  
- [Navigation2 Installation](https://navigation.ros.org/getting_started/index.html)  
- [TurtleBot3 Setup Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)  

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

### 2️⃣ Launch the multi-agent example
```bash
ros2 launch disaster_response_swarm multi_agent_demo.launch.py
```
This will:
- Spawn multiple robots in **Gazebo**
- Start SLAM + Nav2 for each robot
- Assign unique namespaces (`/tb3_0`, `/tb3_1`, …)

---

## Visualization

Launch RViz in another terminal:
```bash
rviz2
```
**Suggested displays**:
- **TF**: View transforms per robot namespace
- **RobotModel**: Show 3D models (`/tb3_X/robot_description`)
- **LaserScan**: View LIDAR scans (`/tb3_X/scan`)
- **Map**: Occupancy grid from SLAM (`/tb3_X/map`)

📚 References: [RViz2 Documentation](https://docs.ros.org/en/foxy/Tutorials/Intermediate/RViz2-Configuration.html)

---

## Customization

| What to change              | How to do it |
|-----------------------------|--------------|
| **Number of robots**        | Edit `multi_agent_demo.launch.py` list of robots |
| **Start positions**         | Modify pose parameters in launch file |
| **SLAM/Nav2 parameters**    | Adjust YAML files in `params/` |
| **Robot model**             | Change `TURTLEBOT3_MODEL` to `burger` or `waffle` |

*Future updates will move these settings into external config files for easier changes.*

---

## Upcoming Features
- **Map merging** for global coordination ([map_merge](https://github.com/hrnr/m-explore))
- Complex disaster world environments
- GUI for easier configuration
- Support for heterogeneous robot teams (ground + aerial)

---

## Contributing
We welcome:
- Feature suggestions
- Bug reports
- Pull requests

Please open an [issue](https://github.com/AmirMohaddesi/disaster_response_swarm/issues) before major changes.

📚 Reference: [ROS 2 Developer Guide](https://docs.ros.org/en/humble/Contributing/)

---

## License
This project is licensed under the [MIT License](LICENSE).

---

**Happy exploring — and may your robots always find the safest path!**
