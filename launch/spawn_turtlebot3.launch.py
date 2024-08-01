from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    def spawn_robot(context, *args, **kwargs):
        namespace = LaunchConfiguration('namespace').perform(context)
        x_pose = LaunchConfiguration('x_pose').perform(context)
        y_pose = LaunchConfiguration('y_pose').perform(context)
        z_pose = LaunchConfiguration('z_pose').perform(context)

        spawn_turtlebot_script_path = os.path.join(
            get_package_share_directory('axperiment'), 'axperiment_scripts', 'spawn_turtlebot.py'
        )

        return [
            ExecuteProcess(
                cmd=['python3', spawn_turtlebot_script_path],
                additional_env={
                    'NAMESPACE': namespace,
                    'X_POSE': x_pose,
                    'Y_POSE': y_pose,
                    'Z_POSE': z_pose
                },
                output='screen'
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument('namespace', description='Namespace for the robot'),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='X position of the robot'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Y position of the robot'),
        DeclareLaunchArgument('z_pose', default_value='0.01', description='Z position of the robot'),
        OpaqueFunction(function=spawn_robot)
    ])