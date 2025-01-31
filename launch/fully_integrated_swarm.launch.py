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
        'nav2_multirobot_params_all_copy.yaml'
    )

    rviz_yaml_file = os.path.join(
        disaster_pkg_dir,
        'rviz',
        'namespaced_rviz_config.rviz'
    )

    map_merge_rviz_file = os.path.join(
        disaster_pkg_dir,
        'rviz', 
        'merge_map.rviz'
    )

    map_merge_params_file = os.path.join(
        disaster_pkg_dir,
        'config', 
        'map_merge_params.yaml'
    )

    
    map_yaml_file = os.path.join(
        '/home/amix/ros_ws/disaster_response_swarm/',
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
        
        # rewritten_yaml = RewrittenYaml(
        #     source_file=nav2_yaml_file,
        #     param_rewrites=param_substitutions,
        #     convert_types=True
        # )

        # Open the file in read mode
        with open(sdf_file_path, 'r') as file:
            # Read the entire content of the file
            file_content = file.read()

        # Replace the old text with the new text
        new_content = file_content.replace('<robot_namespace>', robot_namespace)

        # Open the file in write mode and write the new content
        new_sdf_file = 'models/' + robot_namespace + '_model.sdf'
        with open(new_sdf_file, 'w') as file:
            file.write(new_content)

        # Robot spawn command with model path
        spawn_entity_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'disaster_response_swarm', 'spawn_robot_server',
                '-urdf', new_sdf_file,
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


        # static_transform_publisher_1 = Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher_1',
        #     namespace=robot_namespace,
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': use_sim_time
        #     }],
        #     remappings=remappings,
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--roll', '0', '--pitch', '0', '--yaw', '0',
        #         '--frame-id', 'map', '--child-frame-id', 'odom'
        #     ]
        # )

        # static_transform_publisher_2 = Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher_2',
        #     namespace=robot_namespace,
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': use_sim_time
        #     }],
        #     remappings=remappings,
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--roll', '0', '--pitch', '0', '--yaw', '0',
        #         '--frame-id', 'odom', '--child-frame-id', 'base_footprint'
        #     ]
        # )

        robot_nav = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(disaster_pkg_dir,
                                                        'launch', 
                                                        'localization_navigation_launch.py')),
            launch_arguments={
                'namespace': robot_namespace,
                'use_namespace': 'True',
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'params_file': nav2_yaml_file,
                'autostart': 'True',
                'use_composition': 'False',
                'container_name': 'nav2_container',
                'use_respawn': 'False',
            }.items()
        )

        # robot_nav = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([turtlebot_nav2_prefix]),
        #     launch_arguments={
        #         'slam': 'False',
        #         'namespace': robot_namespace,
        #         'use_namespace': 'True',
        #         'rviz_config': rviz_yaml_file,
        #         'map' : map_yaml_file,
        #         'autostart': 'True',
        #         'use_sim_time': use_sim_time,
        #         'params_file' : rewritten_yaml,
        #         'use_composition': 'True',
        #         'use_respawn': 'False',
        #         'use_simulator': 'False',
        #         'spawn': 'False',                        #added to condition spawn
        #         'use_robot_state_pub': 'False',
        #         'use_rviz': 'False',
        #         'headless': 'False'
        #     }.items()
        # )
        
        # lifecycle_manager_cmd = Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_localization',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time},
        #                 {'autostart': True},
        #                 {'node_names': ['amcl']}]
        # )


        # Robot spawn command with model path
        initial_pose_publisher = Node(
            package='disaster_response_swarm', 
            executable='initial_pose_publisher',  # The name of your executable
            name='initial_pose_publisher',        # Name of the node
            namespace=robot_namespace,
            output='screen',              # Output to screen
            parameters=[{'robot_namespace': robot_namespace}],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
        )

        graph_construction_node = Node(
            package='disaster_response_swarm',
            executable='graph_construction_node',
            name='unified_path_only_node',
            namespace=robot_namespace,
            output='screen',
            parameters=[{'robot_namespace': robot_namespace}],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
        )

        start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            namespace=robot_namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': [
                            'amcl',
                            'map_server',
                            'velocity_smoother',
                            'controller_server',
                            'collision_monitor',
                            'planner_server'
                        ]}])

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
                'use_lifecycle_manager': 'True',
                'use_sim_time': use_sim_time,
                'use_namespace': 'True',
                'global_frame': 'map',
                'save_map_time': 0.0,  # Disable automatic map saving
                'lifetime': 3600.0,  # Keep the map data alive for 1 hour
                'publish_rate': 2.0,  # Publish the map at 2 Hz
                'map_save_dir': '/home/amix/ros_ws/disaster_response_swarm/tmp',  # Save maps to a different directory
                }
            ],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
                ("/scan", "scan"),
                ("/odom", "odom"),
                ("/map", "local_map"),
                ("/map_metadata", "map_metadata"),
                ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
                ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
            ],

        )

        robot_ld = GroupAction([
            spawn_entity_cmd,
            rsp_cmd,
            initial_pose_publisher,
            # static_transform_publisher_1,
            # static_transform_publisher_2,
            robot_nav,
            # graph_construction_node,
            # start_lifecycle_manager_cmd,
            robot_slam,
            robot_rviz
        ])

        simulation_ld.append(robot_ld)

    map_merge_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='map_merge_rviz',
        output='screen',
        arguments=['-d', map_merge_rviz_file],
        parameters=[{'use_sim_time': use_sim_time}]
    ) 


    # Robot spawn command with model path
    merge_map_cmd = Node(
            package='disaster_response_swarm',  # Replace with your package name
            executable='merge_map_node',  # The name of your executable
            name='merge_map_node',        # Name of the node
            output='screen',              # Output to screen
            parameters=[],
            remappings=[],
        )
    
    # simulation_ld.append(merge_map_cmd)
    # simulation_ld.append(map_merge_rviz)

    return  LaunchDescription(simulation_ld)