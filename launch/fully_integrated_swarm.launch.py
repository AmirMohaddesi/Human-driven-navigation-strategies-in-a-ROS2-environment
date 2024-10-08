# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable
# from launch.substitutions import LaunchConfiguration
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.actions import Node
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from nav2_common.launch import RewrittenYaml



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

    turtlebot_nav2_prefix = os.path.join(
        disaster_pkg_dir,
        'launch', 
        'tb3_simulation_launch.py'
    )

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

    map_merge_rviz_file = os.path.join(
        get_package_share_directory('merge_map'),
        'config', 
        'merge_map.rviz'
    )

    map_yaml_file = os.path.join(
        disaster_pkg_dir,
        'maps',
        'map.yaml'
    )

    # Remapping topics for each robot
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    slam_yaml_file = os.path.join(
        disaster_pkg_dir,
        'config',
        'slam_online_async.yaml'
    )


    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()


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
    
    simulation_ld =[
        # setup_environment,
        gzserver_cmd,
        gzclient_cmd
    ]

    robots = ['robot1', 'robot2']

    poses = {
        'robot1': {'x': 3.0, 'y': 3.0, 'z': 0.01},
        'robot2': {'x': 1.0, 'y': 2.0, 'z': 0.01}
    }

    for robot in robots:
        
        pose = poses[robot]
        robot_namespace = robot + '_ns'

        # Create the RewrittenYaml substitution
        param_substitutions = {
            '<robot_namespace>': robot_namespace
        }
        
        rewritten_yaml = RewrittenYaml(
            source_file=nav2_yaml_file,
            param_rewrites=param_substitutions,
            convert_types=True
        )

        # Robot spawn command with model path
        spawn_entity_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'disaster_response_swarm', 'spawn_robot_server',
                '-urdf', sdf_file_path,
                '-n', robot,
                '-ns', robot_namespace,
                '-namespace', 'true',
                '-use_sim_time', use_sim_time,
                '-x', str(pose['x']), '-y', str(pose['y']), '-z', str(pose['z'])
            ],
            output='screen'
        )


        # Robot state publisher with the model description
        rsp_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                        'robot_description': robot_desc}],
            remappings=remappings
        )

        # Robot state publisher with the model description
        jsp_cmd = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=robot_namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc}],
            remappings=remappings
        )

    
        # Static Transform Publishers as per instructions
        static_transform_publisher_1 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_1',
            namespace=robot_namespace,
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
            namespace=robot_namespace,
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
            namespace=robot_namespace,
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
            namespace=robot_namespace,
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

        robot_nav = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot_nav2_prefix]),
            launch_arguments={
                'slam': 'False',
                'namespace': robot_namespace,
                'use_namespace': 'True',
                'rviz_config': rviz_yaml_file,
                'map' : map_yaml_file,
                'autostart': 'True',
                'use_sim_time': use_sim_time,
                'params_file' : rewritten_yaml,
                'use_composition': 'False',
                'use_respawn': 'True',
                'use_simulator': 'False',
                'spawn': 'False',                        #added to condition spawn
                'use_robot_state_pub': 'False',
                'use_rviz': 'False',
                'headless': 'False'
            }.items()
        )

        robot_rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
            launch_arguments={
                'namespace': robot_namespace,
                'use_namespace': 'True',
                'use_sim_time': use_sim_time,
                'rviz_config': rviz_yaml_file}.items())

        robot_slam = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            namespace=robot_namespace,
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

        robot_ld = GroupAction([
            spawn_entity_cmd,
            rsp_cmd,
            jsp_cmd,
            static_transform_publisher_1,
            static_transform_publisher_2,
            static_transform_publisher_3,
            static_transform_publisher_4,
            robot_nav,
            robot_slam,
            robot_rviz
        ])

        simulation_ld.append(robot_ld)


    map_merge_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', map_merge_rviz_file],
        parameters=[{'use_sim_time': True}]
    ) 
    
    map_merge_rviz = Node(
        package='merge_map',
        executable='merge_map',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ("/map1", "/robot1_ns/map"),
            ("/map2", "/robot2_ns/map"),
        ]
    )

    simulation_ld.append(map_merge_node)
    simulation_ld.append(map_merge_rviz)

    return  LaunchDescription(simulation_ld)