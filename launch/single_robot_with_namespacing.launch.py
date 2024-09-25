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
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import LifecycleNode


import os

def generate_launch_description():
    
        # Get the urdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')
    disaster_pkg_dir = get_package_share_directory('disaster_response_swarm')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    world_file = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_house.world')


    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'

    urdf_file_path = os.path.join(
        disaster_pkg_dir,
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

    rviz_yaml_file = os.path.join(
        disaster_pkg_dir,
        'rviz',
        'namespaced_rviz_config.rviz'
    )

    map_yaml_file = os.path.join(
        disaster_pkg_dir,
        'maps',
        'map.yaml'
    )

    # Remapping topics for each robot
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }

    # Rewritten Yaml for each robot namespace
    robot1_params = RewrittenYaml(
        source_file=nav2_yaml_file,
        root_key='robot1_ns',
        param_rewrites=param_substitutions,
        convert_types=True)
    
    slam_yaml_file = os.path.join(
        disaster_pkg_dir,
        'config',
        'slam_online_async.yaml'
    )

    print("urdf_file_path : {}".format(urdf_file_path))


    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # rsp_params = {'robot_description': robot_desc}


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
            '-use_sim_time', use_sim_time,
            '-x', '3.0', '-y', '3.0', '-z', '0.01'
        ],
        output='screen'
    )

    # Robot state publisher with the model description
    rsp_cmd1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot1_ns',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_desc}],
        remappings=remappings
    )

    # Robot state publisher with the model description
    jsp_cmd1 = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='robot1_ns',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc}],
        remappings=remappings
    )

    # Static Transform Publishers as per instructions
    map_stp = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=remappings,
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'map'
        ]
    )
    
    # Static Transform Publishers as per instructions
    static_transform_publisher_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_1',
        namespace='robot1_ns',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=remappings,
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
        namespace='robot1_ns',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=remappings,
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
        namespace='robot1_ns',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=remappings,
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
        namespace='robot1_ns',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=remappings,
        arguments=[                       
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '-1.5708', '--yaw', '0',
            '--frame-id', 'camera_link', '--child-frame-id', 'camera_rgb_optical_frame'
        ]
    )
    
    # static_transform_publisher_5 = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher_5',
    #     namespace='robot1_ns',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }],
    #     remappings=remappings,
    #     arguments=[                       
    #         '--x', '0', '--y', '0', '--z', '0',
    #         '--roll', '0', '--pitch', '0', '--yaw', '0',
    #         '--frame-id', 'map', '--child-frame-id', 'base_scan'
    #     ]
    # )

    # Include Navigation2 launch file without AMCL and map server
    turtlebot_nav2_prefix = PathJoinSubstitution([
        # get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py'])
        disaster_pkg_dir, 'launch', 'tb3_simulation_launch.py'])
    

    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot_nav2_prefix]),
        launch_arguments={
            'slam': 'False',
            'namespace': 'robot1_ns',
            'use_namespace': 'True',
            'rviz_config': rviz_yaml_file,
            'map' : map_yaml_file,
            'autostart': 'True',
            'use_sim_time': use_sim_time,
            'params_file' : nav2_yaml_file,
            'use_composition': 'False',
            'use_respawn': 'True',
            'use_simulator': 'False',
            'spawn': 'False',                        #added to condition spawn
            'use_robot_state_pub': 'False',
            'use_rviz': 'False',
            'headless': 'False'
        }.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={
            'namespace': 'robot1_ns',
            'use_namespace': 'True',
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_yaml_file}.items())

    # # Include SLAM Toolbox launch file
    # slam_toolbox_prefix = PathJoinSubstitution([
    #     get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'])
    
    # slam = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([slam_toolbox_prefix]),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'slam_params_file' : slam_yaml_file
    #     }.items(),

    # )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='robot1_ns',
        parameters=[
            slam_yaml_file,
            {
            'use_lifecycle_manager': 'False',
            'use_sim_time': use_sim_time,
            'use_namespace': 'True'
            }
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/scan", "scan"),
            ("/odom", "odom"),
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
            ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
        ],

    )

    return LaunchDescription([
        # setup_environment,
        gzserver_cmd,
        gzclient_cmd,
        spawn_entity_cmd1,
        rsp_cmd1,
        jsp_cmd1,
        # map_stp,
        # GroupAction([
            # PushRosNamespace('robot1_ns'),
            # spawn_entity_cmd1,
            # rsp_cmd1,
        static_transform_publisher_1,
        static_transform_publisher_2,
        static_transform_publisher_3,
        static_transform_publisher_4,
            # navigation2,
            # slam
        # ]),
        navigation2,
        slam,
        rviz
    ])

