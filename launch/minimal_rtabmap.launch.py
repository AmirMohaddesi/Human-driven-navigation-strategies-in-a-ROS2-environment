from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # RTAB-Map launch as Node
    rtabmap_cmd = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',       # Robot's base frame
            'odom_frame_id': 'odom',       # Odometry frame
            'map_frame_id': 'map',         # Map frame
            'subscribe_depth': False,      # Assume RGB-D camera for simplicity
            'subscribe_rgb': True,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom', '/robot1/odom')       # Ensure odom frame is correctly subscribed to
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        rtabmap_cmd
    ])