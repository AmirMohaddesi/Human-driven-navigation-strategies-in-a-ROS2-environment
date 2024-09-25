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
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    
        # Get the urdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')
    disaster_pkg_dir = get_package_share_directory('disaster_response_swarm')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
    world_file = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_house.world')

    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    # sdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '/model.sdf'
    urdf_file_path = os.path.join(
        turtlebot3_description_dir,
        'urdf',
        urdf_file_name
    )
    sdf_file_path = os.path.join(
        disaster_pkg_dir,
        'models',
        model_folder,
        'model.sdf'
    )

    nav2_yaml_file = os.path.join(
        disaster_pkg_dir,
        'config',
        'nav2_multirobot_params_all.yaml'
    )

    slam_yaml_file = os.path.join(
        disaster_pkg_dir,
        'config',
        'slam_online_async.yaml'
    )

    print("urdf_file_path : {}".format(urdf_file_path))


    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}


    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world' : world_file, 
            'use_sim_time': use_sim_time
        }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzclient.launch.py'))
    )

    # Robot spawn command with model path
    spawn_entity_cmd1 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'disaster_response_swarm', 'spawn_robot_server',
            '-urdf', sdf_file_path,
            '-n', 'robot1',
            '-ns', 'robot1_ns',
            '-namespace', 'true',
            # '-use_sim_time', 'True',
            '-x', '3.0', '-y', '3.0', '-z', '0.01'
        ],
        output='screen'
    )

    # Robot state publisher with the model description
    rsp_cmd1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # namespace='robot1_ns',
        output='screen',
        parameters=[rsp_params, {'use_sim_time': use_sim_time}])
    
    # Static Transform Publishers as per instructions
    static_transform_publisher_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_1',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ]
    )

    static_transform_publisher_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_2',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'odom', '--child-frame-id', 'base_footprint'
        ]
    )
    
    static_transform_publisher_3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_3',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[           
            '--x', '0', '--y', '0.1', '--z', '0.3',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint', '--child-frame-id', 'camera_link'
        ]
    )

    static_transform_publisher_4 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_4',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[                       
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '-1.5708', '--yaw', '0',
            '--frame-id', 'camera_link', '--child-frame-id', 'camera_rgb_optical_frame'
        ]
    )

    # Include Navigation2 launch file without AMCL and map server
    turtlebot_nav2_prefix = PathJoinSubstitution([
        get_package_share_directory('nav2_bringup'), 'launch', 'tb3_simulation_launch.py'])
    

    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot_nav2_prefix]),
        launch_arguments={
            # 'slam': 'True',
            # 'namespace': 'robot1_ns',
            # 'use_namespace': 'True',
            'map' : 'maps/map.yaml',
            'use_sim_time': use_sim_time,
            'params_file' : nav2_yaml_file,
            # 'use_composition': 'True',
            'use_respawn': 'True'
        }.items()
    )

    # Include SLAM Toolbox launch file
    slam_toolbox_prefix = PathJoinSubstitution([
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'])
    
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_toolbox_prefix]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file' : slam_yaml_file
        }.items()
    )

    return LaunchDescription([
        # setup_environment,
        gzserver_cmd,
        gzclient_cmd,
        # spawn_entity_cmd1,
        # rsp_cmd1,
        # GroupAction([
        #     PushRosNamespace('robot1_ns'),
            # rsp_cmd1
        static_transform_publisher_1,
        static_transform_publisher_2,
        static_transform_publisher_3,
        static_transform_publisher_4,
            # navigation2,
            # slam
        # ]),
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