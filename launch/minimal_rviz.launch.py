from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to RViz config file
    rviz_config_path = os.path.join(
        get_package_share_directory('disaster_response_swarm'), 'rviz', 'minimal_rviz_config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        rviz_node
    ])