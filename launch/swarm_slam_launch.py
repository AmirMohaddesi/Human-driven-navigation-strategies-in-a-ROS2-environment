import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package share directory for your package and Swarm-SLAM
    swarm_slam_pkg_dir = get_package_share_directory('Swarm-SLAM')
    disaster_response_pkg_dir = get_package_share_directory('disaster_response_swarm')

    # Define the path to your custom world file and map file
    world_file = os.path.join(disaster_response_pkg_dir, 'worlds', 'disaster_world.world')
    map_file = os.path.join(disaster_response_pkg_dir, 'maps', 'map.yaml')

    # Use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo server launch
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'use_sim_time': use_sim_time}.items()
    )

    # Gazebo client launch
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )

    # Swarm-SLAM node
    swarm_slam_node = Node(
        package='swarm_slam',
        executable='swarm_slam_node',
        name='swarm_slam_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=[
            # Example remappings
            ('/robot1/scan', '/robot1_scan'),
            ('/robot2/scan', '/robot2_scan')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        gzserver_cmd,
        gzclient_cmd,
        swarm_slam_node,
    ])