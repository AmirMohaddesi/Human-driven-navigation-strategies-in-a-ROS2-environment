# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable
# from launch.substitutions import LaunchConfiguration
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.actions import Node
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Set the TURTLEBOT3_MODEL environment variable to 'waffle'
    setup_environment = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle')

    disaster_response_prefix = get_package_share_directory('disaster_response_swarm')

    # Include the robot interface launch file
    turtlebot3_bringup_prefix = PathJoinSubstitution([
        get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py'])
    

    robot_interfaces = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot3_bringup_prefix])
    )

    # Include Navigation2 launch file without AMCL and map server
    turtlebot_nav2_prefix = PathJoinSubstitution([
        get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py'])
    

    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot_nav2_prefix]),
        launch_arguments={
            'use_sim_time': 'True',
            'map' : 'maps/map.yaml',
            'slam': 'True',
            'params_file' : 'config/nav2_params.yaml'
        }.items()
    )

    # Include SLAM Toolbox launch file
    slam_toolbox_prefix = PathJoinSubstitution([
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'])
    
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_toolbox_prefix]),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file' : PathJoinSubstitution([disaster_response_prefix , 'maps','map.yaml'])
        }.items()
    )

    return LaunchDescription([
        setup_environment,
        robot_interfaces,
        navigation2,
        slam
    ])

# def generate_launch_description():
#     # Directory paths
#     slam_toolbox_pkg_dir = get_package_share_directory('slam_toolbox')
#     turtlebot3_gazebo_pkg_dir = get_package_share_directory('turtlebot3_gazebo')
#     nav2_bringup_pkg_dir = get_package_share_directory('nav2_bringup')
#     use_sim_time = LaunchConfiguration('use_sim_time', default='true')

#     # rtabmap_launch_pkg_dir = get_package_share_directory('rtabmap_launch')

#     # File paths
#     nav2_params_file = os.path.join(nav2_bringup_pkg_dir, 'params', 'nav2_params.yaml')
#     # rviz_config_file = os.path.join(nav2_bringup_pkg_dir, 'rviz', 'nav2_default_view.rviz')

#     return LaunchDescription([
#         # # Set environment variable
#         SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),

#         # Launch Gazebo Server and Client
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(turtlebot3_gazebo_pkg_dir, 'launch', 'turtlebot3_world.launch.py'))
#         ),

#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(nav2_bringup_pkg_dir, 'launch', 'tb3_simulation_launch.py'))
#         ),

#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(slam_toolbox_pkg_dir, 'launch', 'online_async_launch.py'))
#         )

#         # # Teleoperation KeyBoard Node
#         # ExecuteProcess(
#         #     cmd=['ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard'],
#         #     output='screen'
#         # ),
        
#         # Static Transform Publishers as per instructions
#         # ExecuteProcess(
#         #     cmd=[
#         #         'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
#         #         '--x', '0', '--y', '0', '--z', '0',
#         #         '--roll', '0', '--pitch', '0', '--yaw', '0',
#         #         '--frame-id', 'map', '--child-frame-id', 'odom'
#         #     ],
#         #     output='screen'
#         # ),
#         # ExecuteProcess(
#         #     cmd=[
#         #         'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
#         #         '--x', '0', '--y', '0', '--z', '0',
#         #         '--roll', '0', '--pitch', '0', '--yaw', '0',
#         #         '--frame-id', 'odom', '--child-frame-id', 'base_footprint'
#         #     ],
#         #     output='screen'
#         # ),
#         # TimerAction(
#         #     period=2.0,
#         #     actions=[
#         #         ExecuteProcess(
#         #             cmd=[
#         #                 'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
#         #                 '--x', '0', '--y', '0.1', '--z', '0.3',
#         #                 '--roll', '0', '--pitch', '0', '--yaw', '0',
#         #                 '--frame-id', 'base_footprint', '--child-frame-id', 'camera_link'
#         #             ],
#         #             output='screen'
#         #         ),
#         #         ExecuteProcess(
#         #             cmd=[
#         #                 'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
#         #                 '--x', '0', '--y', '0', '--z', '0',
#         #                 '--roll', '0', '--pitch', '-1.5708', '--yaw', '0',
#         #                 '--frame-id', 'camera_link', '--child-frame-id', 'camera_rgb_optical_frame'
#         #             ],
#         #             output='screen'
#         #         )
#         #     ]
#         # ),

#         # # RTAB-Map
#         # TimerAction(
#         #     period=5.0,
#         #     actions=[ExecuteProcess(
#         #         cmd=[
#         #             'ros2', 'launch', 'rtabmap_launch', 'rtabmap.launch.py',
#         #             'visual_odometry:=false',
#         #             'frame_id:=base_footprint',
#         #             'subscribe_scan:=true',
#         #             'depth:=false',
#         #             'approx_sync:=true',
#         #             'odom_topic:=/odom',
#         #             'scan_topic:=/scan',
#         #             'qos:=2',
#         #             'args:="-d --RGBD/NeighborLinkRefining true --Reg/Strategy 1"',
#         #             'use_sim_time:=true',
#         #             'rviz_config:=' + rviz_config_file  # Using same RViz config
#         #         ],
#         #         output='screen'
#         #     )]
#         # ),

#     ])