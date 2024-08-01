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

    # Robot spawn command
    spawn_entity_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-file', sdf_file,
            '-entity', 'turtlebot3_waffle_pi',
            '-x', '0.0', '-y', '0.0', '-z', '0.01',
            '-robot_namespace', 'robot1'
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
    use_sim_time = {'use_sim_time': 'true'}

    # RTAB-Map launch as Node
    rtabmap_cmd = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',       # Robot's base frame
            'odom_frame_id': 'odom',       # Odometry frame
            'map_frame_id': 'map',         # Map frame
            'subscribe_depth': False,      # Assume RGB-D camera for simplicity
            'subscribe_rgb': True,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('rgb/image', '/robot1/camera/rgb/image_raw'),
            ('rgb/camera_info', '/robot1/camera/rgb/camera_info'),
            ('odom', '/robot1/odom')       # Ensure odom frame is correctly subscribed to
        ]
    )

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,  # Comment out if not necessary to save resources
        spawn_entity_cmd,
        rviz_node,
        rtabmap_cmd
    ])