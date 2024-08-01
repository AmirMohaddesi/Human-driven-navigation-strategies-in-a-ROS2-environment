from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_pkg_dir = get_package_share_directory('turtlebot3_gazebo')
    disaster_pkg_dir = get_package_share_directory('disaster_response_swarm')
    
    models_path = os.path.join(disaster_pkg_dir, 'models', 'burgerplus.sdf')
    world_file = os.path.join(turtlebot3_gazebo_pkg_dir, 'worlds', 'turtlebot3_house.world')

    if not os.path.isfile(models_path):
        raise FileNotFoundError(f"Model file does not exist: {models_path}")

    # Gazebo server launch
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )

    # Gazebo client launch
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzclient.launch.py'))
    )

    # Robot spawn command with model path
    spawn_entity_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-file', models_path,
            '-entity', 'my_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.01',
            '-robot_namespace', 'robot1'
        ],
        output='screen'
    )

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,  # Comment out if not necessary to save resources
        spawn_entity_cmd
    ])