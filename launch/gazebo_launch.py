import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the package share directory for "gazebo_ros"
    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')

    # Define the path to your custom world file
    world_file = os.path.join(
        get_package_share_directory('disaster_response_swarm'),
        'worlds',
        'disaster_world.world'
    )

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
    ])