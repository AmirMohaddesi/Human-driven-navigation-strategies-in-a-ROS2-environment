from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set the TurtleBot3 model type
    turtlebot3_model = 'waffle'
    use_sim_time = 'true'
    
    # Get the directory paths
    turtlebot3_gazebo_pkg_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_pkg_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_pkg_dir = get_package_share_directory('slam_toolbox')
    
    rviz_config_file = os.path.join(nav2_bringup_pkg_dir, 'rviz', 'nav2_default_view.rviz')

    if not os.path.isfile(rviz_config_file):
        raise FileNotFoundError(f"RViz config file not found: {rviz_config_file}")

    return LaunchDescription([
        # Set environment variable for TurtleBot3 model
        SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model),

        # Launch Gazebo with TurtleBot3 world
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'],
            output='screen'
        ),

        # Start Navigation2 stack
        ExecuteProcess(
            cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', f'use_sim_time:={use_sim_time}'],
            output='screen'
        ),

        # Start SLAM Toolbox
        ExecuteProcess(
            cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py', f'use_sim_time:={use_sim_time}'],
            output='screen'
        ),

        # Start RViz with specified configuration file
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_config_file],
            output='screen'
        )
    ])