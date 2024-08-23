from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_pkg_dir = get_package_share_directory('turtlebot3_gazebo')
    disaster_pkg_dir = get_package_share_directory('disaster_response_swarm')
    
    world_file = os.path.join(turtlebot3_gazebo_pkg_dir, 'worlds', 'turtlebot3_house.world')
    sdf_file = os.path.join(disaster_pkg_dir, 'models', 'turtlebot3_waffle_pi', 'model.sdf')

    if not os.path.isfile(sdf_file):
        raise FileNotFoundError(f"SDF file does not exist: {sdf_file}")

    # Gazebo server launch
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )

    # Gazebo client launch (GUI)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzclient.launch.py'))
    )

    # Ensure consistent instance of Robot Specification
    robot_name = 'turtlebot3_waffle_pi'
    namespace = 'robot1'
    x_pos = '0.0'  # Initial position

    # Robot spawn command
    spawn_entity_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-file', sdf_file,
            '-entity', robot_name,
            '-x', x_pos, '-y', '0.0', '-z', '0.01',
            '-robot_namespace', namespace
        ],
        output='screen'
    )

    # Node for RViz2
    rviz_config_path = os.path.join(disaster_pkg_dir, 'rviz', 'minimal_rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Use simulation time for all nodes
    use_sim_time_param = True

    # RTAB-Map launch as Node
    rtabmap_cmd = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        namespace=namespace,
        parameters=[{
            'frame_id': 'base_link',       # Robot's base frame
            'odom_frame_id': 'odom',       # Odometry frame
            'map_frame_id': 'map',         # Map frame
            'subscribe_depth': False,      # Assume RGB-D camera for simplicity
            'subscribe_rgb': True,
            'use_sim_time': use_sim_time_param  # Actual boolean
        }],
        remappings=[
            ('rgb/image', f'/{namespace}/camera/rgb/image_raw'),
            ('rgb/camera_info', f'/{namespace}/camera/rgb/camera_info'),
            ('odom', f'/{namespace}/odom')  # Ensure odom frame is correctly subscribed to
        ]
    )

    # Nav2 launch as Node
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'true',   # Passed as strings
            'slam': 'false',           # Passed as strings
            'map': '',
            'use_sim_time': 'true',    # Passed as strings
            'params_file': os.path.join(disaster_pkg_dir, 'params', 'navigation2_params.yaml'),
            'autostart': 'true'        # Passed as strings
        }.items()
    )

    return LaunchDescription([
        gzserver_cmd,
        # gzclient_cmd,  # Comment out if not necessary to save resources
        spawn_entity_cmd,
        rviz_node,
        rtabmap_cmd,
        nav2_cmd
    ])