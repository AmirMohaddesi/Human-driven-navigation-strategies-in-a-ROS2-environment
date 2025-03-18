ROS2 Multi-Agent Disaster Response Platform
This repository contains a ROS 2 (Humble Hawksbill) based multi-robot platform designed for disaster response scenarios. It allows you to deploy multiple TurtleBot3 robots in simulation to perform tasks like simultaneous localization and mapping (SLAM) and autonomous navigation in a coordinated way. The platform is built on ROS2 and incorporates Nav2 for navigation and the Slam Toolbox for mapping. The documentation below provides a step-by-step guide for installation, setup, running a basic multi-robot example, customization options, and a glimpse into future extensions. It is intended to be clear and accessible, even if you have minimal experience with Ubuntu or ROS2.
Installation Guide
Follow these instructions to set up ROS2 Humble and all necessary dependencies on Ubuntu 22.04. By the end of this section, you will have ROS2 and the required packages (Slam Toolbox, Nav2, RViZ, Gazebo, TurtleBot3) installed and configured.
Install ROS 2 Humble:
Ensure you are running Ubuntu 22.04 (Jammy). Add the ROS2 apt repository to your system and install ROS2 Humble. It’s recommended to install the ROS 2 desktop variant which includes RViZ and other useful tools​
EMANUAL.ROBOTIS.COM
​
EMANUAL.ROBOTIS.COM
.
Set up the ROS2 apt sources and GPG key (see the [ROS2 installation guide] for detailed steps). Then update your package index:
bash
Copy
Edit
sudo apt update
Install ROS2 Humble (desktop edition):
bash
Copy
Edit
sudo apt install ros-humble-desktop
After installation, source the ROS2 setup script to load the environment:
bash
Copy
Edit
source /opt/ros/humble/setup.bash
For convenience, you can add this source line to your ~/.bashrc so that it’s automatically sourced in every new terminal​
EMANUAL.ROBOTIS.COM
.
Install required ROS2 packages (dependencies):
Besides the core ROS2, you need to install several packages that the multi-agent platform depends on:
Navigation2 (Nav2) and Nav2 Bringup: These provide the navigation stack (path planning, AMCL, etc.). Install them via apt:
bash
Copy
Edit
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
Reference: This installs Nav2 and example launch files for bringing up navigation​
EMANUAL.ROBOTIS.COM
.
Slam Toolbox: This is used for SLAM (Simultaneous Localization and Mapping) to build maps with the robots’ LIDAR data. Install:
bash
Copy
Edit
sudo apt install ros-humble-slam-toolbox
Gazebo simulator: Install Gazebo (ROS 2 integration) packages so you can simulate robots in a virtual environment:
bash
Copy
Edit
sudo apt install ros-humble-gazebo-*
The wildcard gazebo-* will grab all Gazebo Classic packages for ROS2 Humble (gazebo ROS plugins, sensors, etc.)​
EMANUAL.ROBOTIS.COM
.
TurtleBot3 packages: Install TurtleBot3 simulation and control packages. There are two ways:
Via apt (quick and easy):
bash
Copy
Edit
sudo apt install ros-humble-turtlebot3*
This will install the TurtleBot3-specific packages, including simulations and teleoperation nodes. (On Ubuntu 22.04 with ROS Humble, packages like ros-humble-turtlebot3-gazebo and ros-humble-turtlebot3-simulation should be included in this wildcard.)
Via source (if you need the latest or if apt is not available): Clone the official TurtleBot3 repositories and build them. For example:
bash
Copy
Edit
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/turtlebot3_ws
sudo apt install python3-colcon-common-extensions  # if not already installed
colcon build --symlink-install
Then source ~/turtlebot3_ws/install/setup.bash. (These steps are taken from the official TurtleBot3 manual​
EMANUAL.ROBOTIS.COM
.)
RViZ 2: If you installed the ROS2 desktop package, RViZ 2 is already included. If not, you can install it separately with:
bash
Copy
Edit
sudo apt install ros-humble-rviz2
Environment configuration:
After installing the packages, a few environment variables need to be set for everything to work smoothly.
ROS Environment Setup: If you haven’t added the ROS source command to your ~/.bashrc earlier, do so now:
bash
Copy
Edit
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
This ensures each new shell loads ROS2. (Also add the TurtleBot3 workspace setup if you built from source.)
TURTLEBOT3_MODEL: TurtleBot3 comes in different models (Burger, Waffle, Waffle Pi). You must specify which model to use so that the correct robot description and meshes are loaded. Set an environment variable in your bashrc:
bash
Copy
Edit
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
Here we use burger (the small two-wheeled model) as an example. If you plan to use the Waffle model, set it to waffle. After updating bashrc, run source ~/.bashrc to apply the changes to the current session.
Gazebo model path (optional): If you installed TurtleBot3 via apt, the Gazebo models will be set up automatically. If you built from source, you might need to add their path to GAZEBO_MODEL_PATH. For example:
bash
Copy
Edit
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models" >> ~/.bashrc
(Adjust the path if needed; ensure it points to the directory containing TurtleBot3 models.)
ROS Domain ID (optional): ROS2 uses DDS for communication, which by default will allow multiple machines to talk if on the same network. If you are running multiple different ROS2 systems on one network (e.g., in a lab), you may want to isolate this one by setting a unique domain ID. For example:
bash
Copy
Edit
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc  # 30 is an arbitrary number
This step is not strictly necessary for running the simulation on a single PC (the default domain 0 will work), but it’s a good practice to avoid crosstalk between ROS2 systems​
EMANUAL.ROBOTIS.COM
.
After adding these lines, re-source your bashrc (source ~/.bashrc) or open a new terminal to ensure all environment variables are set. Now your system should be ready with ROS2 Humble and all required dependencies installed.
Fetching and Setting Up the Repository
Next, get the multi-agent platform code and set up your ROS2 workspace.
Create a ROS2 workspace (if you don’t have one):
Decide on a directory for your workspace (commonly ~/ros2_ws or ~/colcon_ws). If it doesn't exist, create it along with a src folder:
bash
Copy
Edit
mkdir -p ~/ros2_ws/src
You can use any path; ~/ros2_ws is just an example.
Clone the repository from GitHub:
Navigate to the src directory of your workspace:
bash
Copy
Edit
cd ~/ros2_ws/src
Use git to clone the repository:
bash
Copy
Edit
git clone https://github.com/<YourUsername>/<your-repo-name>.git
Replace <YourUsername>/<your-repo-name> with the actual GitHub username and repository name for the multi-agent platform. This will download the project’s source code into your workspace.
Build the workspace with colcon:
After cloning, go back to the root of your workspace:
bash
Copy
Edit
cd ~/ros2_ws
Now build the packages using colcon (the build tool for ROS2):
bash
Copy
Edit
colcon build --symlink-install
This will compile the repository and its packages. The --symlink-install option is helpful for development, as it allows easier editing of Python launch files without rebuilding. If the build finishes with no errors, the packages should be ready to use.
Source the workspace setup script:
Once the build is done, you need to source the local setup to overlay this workspace on top of the ROS installation. From the workspace root, run:
bash
Copy
Edit
source install/setup.bash
You should add this line to your ~/.bashrc as well (below the line where you sourced ROS2) so that your workspace is automatically sourced in new terminals:
bash
Copy
Edit
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
Now your environment knows about the new packages. You can verify by running ros2 pkg list | grep <your-package-name> – it should show the packages from the repository. Ensure again that TURTLEBOT3_MODEL is set in this terminal (echo $TURTLEBOT3_MODEL to check; if not, export it as done above).
With the repository cloned and built, and your environment configured, you are ready to launch the multi-robot simulation and see the platform in action.
Running a Minimal Example
This section will guide you through launching a basic scenario with multiple TurtleBot3 agents to verify that everything is set up correctly. In this example, the robots will perform SLAM in a simulated world and use Nav2 for navigation. We will also use RViZ to visualize the robots and their sensor data. Note: Before proceeding, make sure you have sourced the ROS2 environment and your workspace (source /opt/ros/humble/setup.bash and source ~/ros2_ws/install/setup.bash) in every terminal you open, or simply open a new terminal if you added them to your bashrc.
1. Launch the multi-robot simulation
Open a terminal for the simulation launch. Run the launch file that starts the multi-agent demo (this launch file should have been installed as part of the repository). For example:
bash
Copy
Edit
ros2 launch multi_agent_disaster_response multi_agent_demo.launch.py
(The actual package name and launch file might differ; replace multi_agent_disaster_response and multi_agent_demo.launch.py with the correct names from the repository.) This launch will typically do the following:
Spawn multiple robots in Gazebo: It will start Gazebo and spawn the specified number of TurtleBot3 robots into the simulation world. Each robot will be placed at a predefined starting pose (e.g., different corners of the map or different coordinates in the world).
Namespace isolation: Each robot is given a unique namespace (e.g., /tb3_0, /tb3_1, /tb3_2, ...). This means all topics and nodes for robot 0 will be under /tb3_0/..., for robot 1 under /tb3_1/..., etc. Namespacing is crucial for multi-robot setups to avoid topic conflicts when all robots use the same type of sensors and controllers.
Start SLAM and Nav2: The launch file is expected to also start a SLAM Toolbox instance for each robot (mapping their environment) and a Nav2 stack (for navigation) for each robot, within their respective namespaces. This might be done by including other launch files or using ROS2 composable nodes. Each robot will then have its own map being built and its own navigation server running. (If the launch file is a "minimal" launch, it might omit Nav2 – but since this is a disaster response platform, we assume Nav2 is included for autonomy.)
If the launch is successful, you will see a Gazebo window pop up and messages in the terminal indicating that robots and nodes are spawning. Gazebo may take a little time to load the environment and robot models the first time. 

Gazebo simulation with multiple TurtleBot3 robots. After launching, the Gazebo client window will display the robots in the world. In this example environment, four TurtleBot3 robots (white models) are inside a bounded area with green obstacles. Each robot’s LIDAR scan is visible as a blue hue around it, indicating the sensor range. The simulation is now running, and under the hood each robot’s SLAM node is mapping and each Nav2 instance is active (though the robots are stationary until given commands). If SLAM and Nav2 didn’t start automatically: In some setups, you might need to launch SLAM and Nav2 separately for each robot. If the multi_agent_demo.launch.py did not include them, open additional terminals and run:
SLAM Toolbox: For each robot namespace, run the SLAM launch. For example, for robot 0:
bash
Copy
Edit
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=<path_to_slam_params.yaml> namespace:=tb3_0 use_sim_time:=true
Do this for tb3_1, tb3_2, etc., changing the namespace each time. The online_async_launch.py launch file (provided by slam_toolbox) starts the SLAM node in asynchronous mode, allowing the robot to move while mapping. Make sure to use use_sim_time:=true so the SLAM node uses simulation time. You may use a common slam_params.yaml for all, or separate ones per robot (with different map filenames if saving maps).
Nav2 (Navigation2 stack): Similarly, start Nav2 for each robot using the Nav2 bringup launch. For example:
bash
Copy
Edit
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=<path_to_nav2_params.yaml> namespace:=tb3_0 use_sim_time:=true autostart:=true
This will bring up the Nav2 stack (including the planner, controller, AMCL or SLAM toolbox integration, and lifecycle manager) for the robot in namespace tb3_0. Use a suitable nav2_params.yaml configuration; you might have one provided in the repository (perhaps tuned for TurtleBot3). Ensure autostart is set to true so that the Nav2 lifecycle nodes automatically activate. Repeat this for each robot (tb3_1, tb3_2, …) in their own terminal, or alternatively, you can copy the launch file and modify it to spawn multiple Nav2 instances if you prefer an automated approach.
When all these are running, each robot should be publishing its own /tb3_x/map topic (from SLAM) and listening on its own /tb3_x/cmd_vel (for movement commands), etc. At this point, the multi-robot system is up and each robot is independently capable of mapping and navigation within the simulation.
2. Launch RViZ for visualization
In a new terminal, launch RViZ 2 to visualize the robots and their sensor data in real-time. You can use the default RViZ launch or a custom config:
bash
Copy
Edit
ros2 launch rviz2 rviz2.launch.py
This brings up an empty RViZ. You will need to add displays to visualize multiple robots. The repository may include an RViZ configuration file (e.g., multi_robot.rviz) that you can load by running, for example:
bash
Copy
Edit
rviz2 -d <path_to_your_ws/src/your-repo/rviz/multi_robot.rviz>
If such a config is provided, it will automatically set up the typical visuals for you. If not, you can manually add the following in RViZ:
TF: Add a TF display to see the frame transforms. You should see separate TF trees for each robot (e.g., frames tb3_0/odom, tb3_0/base_link, etc., and similarly for tb3_1, etc.). This confirms each robot has its own transform frames published.
Robot Model: Add one RobotModel display per robot, and set each to the robot’s namespace (e.g., set “Description Topic” to /tb3_0/robot_description for the first robot, etc.). This will show the 3D model of each TurtleBot3 in RViZ.
Laser Scan: Add a LaserScan display for each robot’s scan topic (e.g., /tb3_0/scan, /tb3_1/scan …). This will visualize the LIDAR points. They should correspond to the environment obstacles.
Map (OccupancyGrid): Add a Map display for each robot’s map topic (e.g., /tb3_0/map). As SLAM runs, you will see each map slowly being filled in. By default, each robot’s map is separate. (In future, map merging might combine them, but in this basic setup you’ll see multiple maps if you add all of them.)
Path / Pose / Goal displays: If you send navigation goals (in the next step), you can add Path displays (for /tb3_x/plan) to see the planned path and PoseArray displays for things like particle clouds (if using AMCL), etc. Initially, this isn’t necessary just to observe SLAM.
If the repository provided a preconfigured RViZ file, it likely already has multiple displays set up so you can see all robots at once. Either way, once RViZ is configured, you should see the robots and sensor data similar to how they appear in Gazebo, as well as the maps being drawn in 2D.
3. Control the robots and navigate
Now that the simulation is running and you can visualize it, you can interact with the robots. This platform doesn’t automate the movement of the robots (by default) — they will sit still until you command them. Here are a couple of ways to control them:
Teleoperation (manual control): Use the keyboard teleop node to drive a robot around. ROS2 provides teleop_twist_keyboard and TurtleBot3 provides its own teleop package. For example, to control the first robot:
bash
Copy
Edit
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/tb3_0
This launches a CLI tool that lets you use WASD keys to drive the robot in namespace /tb3_0. Make sure the terminal is focused so your keypresses register. You can open additional terminals to control other robots (/tb3_1, /tb3_2, etc.) simultaneously. Drive the robots around a bit to map different areas. You should see their individual maps grow in RViZ (and in Gazebo, the robots will move accordingly).
Tip: Only drive one robot per teleop instance. The __ns remapping ensures the commands go to the correct robot. Without it, you’d control a non-namespaced robot which doesn’t exist in this simulation.
Autonomous navigation: You can also send goals to the Nav2 stack of each robot. In RViZ, use the “Navigation2 Goal” tool (the icon of a 2D nav goal) to send a goal pose. However, since multiple Nav2 instances are running, you need to ensure the goal is sent to the correct namespace. One way is to run separate RViZ instances, each scoped to a different robot’s namespace (by setting the ROS_NAMESPACE environment variable before launching RViZ, e.g., ROS_NAMESPACE=tb3_0 rviz2 -d nav2_default_view.rviz). Alternatively, you can programmatically send goals with a ROS2 action call to /tb3_x/navigate_to_pose. For simplicity, using teleop or separate RViZ instances might be easier for a beginner.
If you send a goal, that robot’s Nav2 will plan and the robot should start moving along the path. You’ll see the path and the robot moving in RViZ. Each robot’s Nav2 works independently, so they might not be aware of each other — be careful that they don’t physically collide in simulation (Nav2 won’t avoid other robots unless their lasers detect each other as obstacles).
4. Expected outcomes
By running this minimal example, you should observe the following:
Gazebo: Multiple robots are present and can move around. They behave like individual TurtleBot3 units. If you drive them, they respond to commands. Gazebo simulates their sensors (LIDAR, odometry, etc.) and the environment.
SLAM and Maps: Each robot is generating its own map. In RViZ, you’ll see one map per robot (if you added all map displays). They will each cover the area the respective robot has explored. Initially, the map will be all unknown (usually gray) and as the robot’s LIDAR scans detect walls or obstacles, you’ll see free space (white) and occupied cells (black) appear. Note: At this stage, the maps are not merged; e.g., robot 0’s map topic only contains what robot 0 has seen. Merging maps is an advanced feature planned for the future.
Navigation: If you sent any navigation goals, each robot’s Nav2 instance should handle them. You would see the robot move to the goal, and the costmaps in RViZ showing inflation around obstacles. If a robot cannot reach a goal, Nav2’s recovery behaviors might kick in (e.g., it might rotate in place or try a different path). Each robot’s logs (in the terminal where you launched Nav2) will show info about planning and control. Successful navigation is a good sign that the Nav2 configuration is correct.
No conflicts: Thanks to namespacing, the robots shouldn’t interfere with each other’s ROS communications. For instance, you will have topics /tb3_0/cmd_vel and /tb3_1/cmd_vel separate, each robot listening only to its own. The minimal example thus demonstrates the platform’s ability to manage multiple agents concurrently.
If all goes well, you now have a multi-robot simulation running, with each robot mapping and navigating. Feel free to experiment in this setup — drive the robots to different areas, perhaps intentionally make them see overlapping areas, etc. This will give you a feel for how the system operates before moving on to more complex scenarios.
Customization Guide
One of the goals of this platform is to be easily customizable to different scenarios, numbers of robots, and configurations. Currently, some customization requires editing launch files or source code (inline configuration). In the future, we plan to move towards using external configuration files for easier setup changes. This section explains what you can customize now and how, and provides guidance on best practices for managing these configurations.
Current Customization Options (Inline)
At present, many parameters are defined within the code or launch files. You can open the relevant files (usually in the launch/ directory or a config script) to adjust them. Here are common things you might want to change:
Number of robots: The number of TurtleBot3 agents launched is likely defined in a launch file. For example, there may be a Python list or a variable in the launch script that enumerates the robots (with names and maybe starting poses). Look for something like robots = [...] or a number_of_robots variable. By editing this, you can increase or decrease the number of robots spawned​
GITHUB.COM
. Ensure that if you add more robots, you also specify unique namespaces and initial positions for them (to avoid spawning all robots at the exact same spot!). After modifying, save the file and rerun the launch to see the effect.
Robot starting positions and names: In multi-robot simulations, each robot’s initial pose (x, y coordinates and possibly orientation) can usually be configured. In the launch file (or a YAML config, if provided), find where the robot poses are set. This could be in a list of dictionaries like:
python
Copy
Edit
robots = [
    {'name': 'tb3_0', 'x_pose': 0.0, 'y_pose': 0.0, ...},
    {'name': 'tb3_1', 'x_pose': 2.0, 'y_pose': 0.0, ...},
    ...
]
You can edit these values or add/remove entries for more or fewer robots. Make sure each robot has a unique name (which is used as namespace) and distinct start coordinates to prevent collisions at startup. The above is just a conceptual example — refer to the actual structure in the repository’s launch file​
GITHUB.COM
.
World environment: The Gazebo world that is loaded can be changed. By default, the launch might be using a world file like turtlebot3_world.world or a custom disaster scenario world. Search the launch file for .world or a world parameter. If you have a different world file (e.g., a more complex environment or a specific map of a disaster site), you can point the simulation to that file. For instance:
bash
Copy
Edit
ros2 launch multi_agent_disaster_response multi_agent_demo.launch.py world:=<path_to_your.world>
(This works if the launch file declares a world launch argument.) If not, you might have to edit the launch to change the hardcoded world path. Be sure the world file is accessible (you may place custom worlds in the worlds/ directory of the repository or use Gazebo’s default worlds).
SLAM and Nav2 parameters: The behavior of SLAM Toolbox and Nav2 can be tuned via parameter files (usually YAML files). Check the params/ directory in the repository for files like slam_params.yaml or nav2_params.yaml. These contain settings like map resolution, frequency of map updates, Nav2 controller parameters, planner tolerances, etc. For example, in nav2_params.yaml you can adjust the planner type or the tolerances for goal acceptance. If the repository includes default parameter files, you can modify them to suit your needs. Common customizations:
SLAM: Map resolution (increase for finer maps at cost of more memory), whether to save the map periodically, loop closure parameters, etc.
Nav2: Planner type (GridBased vs. other planners), recovery behaviors (you can disable spinning or clearing if not desired), costmap sizes, inflation radius, etc.
Make sure to re-run the launch after changes. Nav2 params can also be reloaded at runtime via reconfiguration, but it’s advanced. Usually, just edit and relaunch for simplicity.
Robot model type: By default we assumed burger model. If you want to use a different TurtleBot3 model (like waffle which is larger and has a camera), you can do so by changing the TURTLEBOT3_MODEL environment variable (as mentioned earlier) and also ensuring any references to the robot’s physical dimensions or sensors are updated. The launch files typically read TURTLEBOT3_MODEL to spawn the correct URDF. So, set it to waffle and then launch — the robots will now be Waffle models (with slightly different size and sensors). Note that if you use Waffle, you might need to install additional packages (e.g., ros-humble-turtlebot3-simulations covers both, but Waffle uses a camera which might require image_common packages if you intend to use it).
Topic names and frame IDs: In multi-robot setups, most topics are under namespaces, but you might want to change some conventions. For example, perhaps instead of tb3_0 you want the namespace to be more descriptive (like alpha, bravo, etc. for robots). You can usually do this by changing the name or namespace values in the launch configurations. Frame IDs (like the base frame or map frame) often incorporate the namespace automatically. Just be cautious: if you change fundamental names, ensure consistency across launch files and config files.
Currently, to apply any of the above changes, you edit the files directly (hence “inline”). Always double-check your changes for typos or syntax errors (especially in YAML or Python, where indentation and commas matter). If something goes wrong (launch crashes or robots don’t spawn), revert your edits and try again.
Future Plans: Configuration Files
We recognize that editing launch files or source code is not the most user-friendly way to configure the system. A key improvement in the pipeline is to move toward external configuration files, so that most (if not all) of the customization can be done without touching the code. Here’s what to expect:
Robot configuration in files: Instead of a hardcoded list of robots in a launch script, we plan to use a JSON or YAML file (e.g., robots.json or robots.yaml) that lists each robot and its properties (name, initial pose, model type). The launch file will then load this file and spawn robots accordingly. This way, adding a robot is as simple as adding a new entry to the config file, and you won’t risk breaking the launch script’s Python code with a syntax error​
GITHUB.COM
. (The repository maintainers have noted this as a potential enhancement, to improve ease of use.)
Scenario and environment configs: Similarly, settings like which world to use, or high-level scenario parameters, could be moved to a config file. For example, a YAML file could specify "world: warehouse_world.world" or "enable_map_merge: false", etc., which the system reads on startup.
Parameter tuning via YAML: While ROS2 already uses YAML for node parameters, we will better separate default parameters from user overrides. We might provide multiple configuration profiles (e.g., a “fast exploration” nav2 param set vs a “safe and slow” one) and allow easy switching between them.
User interface for configuration (long term): Eventually, we envision a simpler interface (perhaps a script or GUI) to select number of robots, type, and scenario without manually editing text files at all.
Until these features are implemented, we suggest using the current inline method carefully and making use of version control (git) to track your changes. This way, when the project updates to use external configs, you can more easily port your custom settings into the new system.
Best Practices for Managing Configurations
Even with the current setup, you can apply some good practices to make customization easier and safer:
Use source control for configuration changes: If you’re comfortable with git, commit your working configurations. For instance, if you tweak the launch file to have 5 robots, commit that change. This makes it easy to revert to a known good state if further edits cause problems. It also helps when pulling updates from the upstream repository (you can merge or reapply your changes).
Document your changes: Keep notes (or comments in the files) about what you changed. This is helpful for future you or for others who might use your fork of the project. For example, if you change the map update rate in slam_params.yaml, add a comment like # changed update_interval from 2.0 to 1.0 for faster map updates.
One thing at a time: When customizing, change one aspect at a time and test. For example, first try adding a robot without changing anything else. Once that works, you might adjust their starting positions. If something breaks, this incremental approach makes it easier to identify which change caused it.
Leverage ROS2 launch arguments: Some launch files might already have configurable arguments (you can check by running ros2 launch <package> <launchfile> --show-args). For instance, there might be an argument for world or rviz_config. Instead of editing the file, you can pass these arguments from the command line or a YAML launch configuration. This is a cleaner way to tweak behavior and is worth using when available.
Stay updated: As this platform evolves, keep an eye on the repository README or changelog. When the shift to configuration files happens, it will likely be documented. Transitioning to that will make life easier. Also, updates may bring new features (or fix bugs) that could affect how you configure the system, so reading update notes is useful.
By following these practices, you can customize the multi-agent system to fit your research or project needs, while minimizing the risk of configuration errors. The maintainers are keen on making this process smoother, so expect it to get easier over time.
Future Extensions
The multi-agent disaster response platform is an active project, and there are many exciting features and improvements on the roadmap. This section provides a brief overview of planned enhancements and how you can contribute or provide feedback.
Map Merging and Global Coordination: Currently, each robot builds its own map. A planned feature is multi-robot map merging, where the maps from individual robots can be combined into a single global map in real-time. This would allow, for example, one robot to use areas explored by another for navigation. Along with map merging, improved coordination strategies (like sharing goals or dividing exploration areas among robots) are under consideration to make the robots work together more intelligently in a disaster scenario.
Heterogeneous Robot Support: While the initial focus is on multiple TurtleBot3 (homogeneous agents), future versions may support different robot types working in tandem (for instance, a drone for aerial survey combined with ground robots). This would require integration of different ROS2 packages (for flight, for manipulation, etc.) and ensuring the platform is flexible in handling varied kinematics and sensor suites.
Enhanced Simulation Environments: To better simulate disaster response, we plan to include more complex Gazebo environments (collapsed buildings, outdoor terrains, etc.). This could involve integration with realistic sensor plugins (e.g., smoke or heat sensors, or camera-based detection of victims) so that the simulation can cover more aspects of a real disaster scenario. Contributions of world files or models representing disaster scenes would be very welcome.
Deployment on Real Robots: Although the main use-case is simulation, the end goal is to have a system that could be deployed on real robot hardware for field tests. Future work will focus on making sure the platform’s communication and coordination can work over real networks (Wi-Fi mesh, etc.), handling real sensor data, and dealing with the unpredictability of physical environments. This may include compatibility with ROS2 multi-machine setups and perhaps integration with ROS2 tools like ROS2 Multirobot (multi-master setups) or OpenRMF for fleet management.
User Interface and Monitoring Tools: We aim to provide better interfaces for operators. This might be a dashboard that shows all robots’ status (battery, positions on a common map, etc.) and allows sending commands to specific robots. ROS2’s web bridge or GUI tools could be leveraged to create a control panel suitable for an emergency response team to oversee the robot team.
Performance Improvements: As the number of robots grows, performance can become an issue (CPU and network usage). We are investigating the use of ROS 2 component nodes and executors to optimize resource use. For example, running multiple Nav2 instances in a single process (with namespacing) to reduce overhead, if the ROS2 middleware allows it. Also, tuning QoS (Quality of Service) settings for topics to ensure robust communication in multi-robot networks is on the agenda.
More Tutorials and Examples: We plan to add scenario-specific examples (for instance, “building collapse scenario”, “forest search and rescue scenario”, etc.) with step-by-step tutorials. These will show how to adapt the platform to different use cases. Also, an expanded troubleshooting section in the documentation will be provided, addressing common issues (like robots not spawning, RViZ not showing data, navigation failing, etc.) along with solutions.
Contributing: We heartily welcome contributions from the community. If you have ideas for improvements or new features, feel free to open an issue on the GitHub repository to discuss them. Bug reports are extremely helpful—if you encounter a problem, please let us know with as much detail as possible so we can fix it. We also accept pull requests: whether it’s for small fixes in documentation or code, or larger additions, we will gladly review them. When contributing code, please follow the style guidelines of the project and test your changes if possible with a simulation run. Feedback: If you’re using this platform for your research or project, we’d love to hear about your experience. Feedback helps us prioritize features and improvements. You can use the GitHub discussions or issue tracker to share your thoughts, or contact the maintainers through the channels listed in the repository.
Thank you for using the ROS2 Multi-Agent Disaster Response Platform! We hope this documentation helped you get started. The field of multi-robot systems for disaster response is challenging but rewarding, and by working together on open platforms like this, we can accelerate development and deployment of these life-saving technologies. Good luck, and happy robotic exploring!
