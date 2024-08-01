import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={'params_file': params_file}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('disaster_response_swarm'),
                'params',
                'nav2_params.yaml'
            ),
            description='Full path to the ROS2 parameters file to use for all launched nodes'
        ),
        nav2_bringup_cmd,
    ])