import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    # LDS_MODEL = os.environ['LDS_MODEL']
    # LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    # usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')
    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('disaster_response_swarm'),  # <--- CHANGE THIS!
            'params',
            TURTLEBOT3_MODEL + '.yaml'))

    print('tb3 parameter: ', os.path.join(
            get_package_share_directory('disaster_response_swarm'),  # <--- CHANGE THIS!
            'params',
            TURTLEBOT3_MODEL + '.yaml'))

    # if LDS_MODEL == 'LDS-01':
    #     lidar_pkg_dir = LaunchConfiguration(
    #         'lidar_pkg_dir',
    #         default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))
    # elif LDS_MODEL == 'LDS-02':
    #     lidar_pkg_dir = LaunchConfiguration(
    #         'lidar_pkg_dir',
    #         default=os.path.join(get_package_share_directory('ld08_driver'), 'launch'))
    #     LDS_LAUNCH_FILE = '/ld08.launch.py'
    # else:
    #     lidar_pkg_dir = LaunchConfiguration(
    #         'lidar_pkg_dir',
    #         default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'


    urdf = os.path.join(
    get_package_share_directory('turtlebot3_description'),
    'urdf',
    urdf_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        # DeclareLaunchArgument(
        #     'usb_port',
        #     default_value=usb_port,
        #     description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzserver.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzclient.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]), so funkt es, aber nur als /scan
        #     beim LDS-02  => ld08_driver
        #     PythonLaunchDescriptionSource([get_package_share_directory('disaster_response_swarm'), '/launch', '/hlds_laser.launch.py']), # <--- CHANGE THIS
        #     PythonLaunchDescriptionSource([get_package_share_directory('disaster_response_swarm'), '/launch', '/ld08.launch.py']), # <--- CHANGE THIS
        #     launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('disaster_response_swarm'), '/launch', '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', TURTLEBOT3_MODEL,
                '-file', urdf,
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.01',
                '-robot_namespace', 'tb3_0'
            ],
            output='screen'
        )
    ])