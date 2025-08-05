# Installation

This guide will walk you through setting up the **disaster_response_swarm** package.

## Requirements
- Ubuntu 22.04
- ROS 2 Humble Hawksbill
- Gazebo (ROS 2 integration)
- TurtleBot3 packages

## Steps

1. Install ROS 2 Humble Desktop:
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

2. Install dependencies:
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup   ros-humble-slam-toolbox ros-humble-gazebo-* ros-humble-turtlebot3* ros-humble-rviz2
```

3. Configure environment variables:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

4. Clone and build the repository:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/AmirMohaddesi/disaster_response_swarm.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
