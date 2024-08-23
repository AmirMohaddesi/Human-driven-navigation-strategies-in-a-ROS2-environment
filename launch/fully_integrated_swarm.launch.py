from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get package directories
    gazebo_pkg_dir = get_package_share_directory('gazebo_ros')
    disaster_response_pkg_dir = get_package_share_directory('disaster_response_swarm')
    turtlebot3_pkg_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_pkg_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_pkg_dir = get_package_share_directory('slam_toolbox')

    # Paths to configuration files
    urdf_file = os.path.join(turtlebot3_pkg_dir, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    nav2_params_file = os.path.join(disaster_response_pkg_dir, 'config', 'nav2_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params_file = os.path.join(disaster_response_pkg_dir, 'config', 'slam_toolbox_config.yaml')
    rviz_config_file = os.path.join(nav2_bringup_pkg_dir, 'rviz', 'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),

        # Start Gazebo without any default robots
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_pkg_dir, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        ),

        # Include spawning launch file for robot 1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(disaster_response_pkg_dir, 'launch', 'spawn_robot_launch.py')),
            launch_arguments={
                'robot_urdf': urdf_file,
                'robot_name': 'robot1',
                'robot_namespace': 'robot1',
                'x': '0.0',
                'y': '0.0',
                'z': '0.0'
            }.items()
        ),

        # Include spawning launch file for robot 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(disaster_response_pkg_dir, 'launch', 'spawn_robot_launch.py')),
            launch_arguments={
                'robot_urdf': urdf_file,
                'robot_name': 'robot2',
                'robot_namespace': 'robot2',
                'x': '2.0',
                'y': '0.0',
                'z': '0.0'
            }.items()
        ),

        # Group for robot 1 - Uncomment when needed
        GroupAction([
            PushRosNamespace('robot1'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='robot1',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[urdf_file]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_pkg_dir, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file,
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(slam_toolbox_pkg_dir, 'launch', 'online_async_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': slam_params_file,
                }.items()
            ),
        ]),

        # Group for robot 2 - Uncomment when needed
        GroupAction([
            PushRosNamespace('robot2'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='robot2',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[urdf_file]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_pkg_dir, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file,
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(slam_toolbox_pkg_dir, 'launch', 'online_async_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': slam_params_file,
                }.items()
            ),
        ]),

        # Launch RViz for visualization - Uncomment when needed
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])