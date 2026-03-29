"""
Gazebo Classic server + client for turtlebot3_house.world.

Mirrors the gzserver/gzclient section of turtlebot3_gazebo/launch/turtlebot3_house.launch.py
(ROBOTIS): same world path resolution and the same launch_arguments to gzserver — only ``world``,
no extra keys. Use this from integrated launches so behavior matches standalone ``ros2 launch
turtlebot3_gazebo turtlebot3_house.launch.py`` for simulation bringup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

from multi_robot_mission_stack.gazebo_launch_env import ensure_gazebo_classic_paths


def generate_launch_description():
    ensure_gazebo_classic_paths()
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world',
    )
    world = os.path.normpath(os.path.abspath(world))

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    f'turtlebot3_house_gazebo.launch.py: gzserver world={world} '
                    f'exists={os.path.isfile(world)}'
                )
            ),
            gzserver_cmd,
            gzclient_cmd,
        ]
    )
