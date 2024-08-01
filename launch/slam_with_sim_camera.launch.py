from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_pkg_dir = get_package_share_directory('turtlebot3_gazebo')
    disaster_pkg_dir = get_package_share_directory('disaster_response_swarm')
    models_path = '/home/hsr/disaster_response_swarm/models/burgerplus.sdf'
    
    # Verify the model path exists
    if not os.path.isfile(models_path):
        raise FileNotFoundError(f"Model file does not exist: {models_path}")
    
    world_file = os.path.join(turtlebot3_gazebo_pkg_dir, 'worlds', 'turtlebot3_house.world')

    # Use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Namespace for the robot'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_model_path_cmd = DeclareLaunchArgument(
        'model',
        default_value=models_path,
        description='Absolute path to the model .sdf file'
    )

    rviz_config_path = os.path.join(disaster_pkg_dir, 'rviz', 'rtabmap_visualization.rviz')

    # Gazebo server launch
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Gazebo client launch
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzclient.launch.py'))
    )

    # Robot spawn command with model path
    spawn_entity_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-file', LaunchConfiguration('model'),
            '-entity', 'my_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.01',
            '-robot_namespace', LaunchConfiguration('namespace')
        ],
        output='screen'
    )

    # RTAB-Map launch as Node
    rtabmap_cmd = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'camera_link',
            'subscribe_depth': False,
            'subscribe_rgb': True,
            'subscribe_stereo': False,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info')
        ]
    )

    # RViz launch as Node
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_model_path_cmd,
        gzserver_cmd,
        gzclient_cmd,
        spawn_entity_cmd,
        rtabmap_cmd,
        rviz_cmd
    ])