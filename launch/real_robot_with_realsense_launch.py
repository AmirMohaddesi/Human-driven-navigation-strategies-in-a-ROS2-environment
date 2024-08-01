from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Define package directories
    disaster_pkg_dir = get_package_share_directory('disaster_response_swarm')
    turtlebot3_gazebo_pkg_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')

    # World file from turtlebot3_gazebo package
    world_file = PathJoinSubstitution(
        [turtlebot3_gazebo_pkg_dir, 'worlds', 'turtlebot3_house.world']
    )

    # Use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Declare arguments
    namespace = LaunchConfiguration('namespace')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    # Gazebo server launch
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'use_sim_time': use_sim_time}.items()
    )

    # Gazebo client launch
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot spawn command
    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(disaster_pkg_dir, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose
        }.items()
    )

    # RealSense camera node (for actual physical camera)
    # Comment or remove this block if not using physical RealSense hardware
    # realsense_camera_cmd = Node(
    #     package='realsense2_camera',
    #     executable='realsense2_camera_node',
    #     name='realsense2_camera',
    #     output='screen',
    #     parameters=[{
    #         'publish_tf': True,
    #         'pointcloud.enable': True,
    #         'depth_module.profile': '640x480x30',
    #         'rgb_camera.profile': '640x480x30',
    #         'enable_sync': True,
    #         'enable_rgbd': True
    #     }]
    # )

    ld = LaunchDescription()

    # Add LaunchArguments
    ld.add_action(DeclareLaunchArgument('namespace', default_value='robot1', description='Namespace for the robot'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='0.0', description='X position of the robot'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='0.0', description='Y position of the robot'))
    ld.add_action(DeclareLaunchArgument('z_pose', default_value='0.01', description='Z position of the robot'))

    # Add all actions
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_robot_cmd)
    # ld.add_action(realsense_camera_cmd) # Comment or remove this line

    return ld