import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Get the package share directory
    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')
    disaster_response_pkg_dir = get_package_share_directory('disaster_response_swarm')

    # Define the path to your custom world file
    world_file = os.path.join(disaster_response_pkg_dir, 'worlds', 'disaster_world.world')

    # Define map file
    map_file = os.path.join(disaster_response_pkg_dir, 'maps', 'map.yaml')

    # Use simulation time
    use_sim_time = 'true'

    # Gazebo server launch
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'use_sim_time': use_sim_time}.items()
    )

    # Gazebo client launch
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzclient.launch.py')
        )
    )

    # Navigation launch
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([disaster_response_pkg_dir, 'params', 'nav2_params.yaml'])
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Path to the world file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation time if true'
        ),
        gzserver_cmd,
        gzclient_cmd,
        nav2_cmd,
    ])