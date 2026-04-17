"""
R1 showcase helper: runtime TF seam + existing P3.2 visibility path.

Use alongside fully_integrated_swarm.launch.py where robot TF is available.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    mission_stack_pkg_dir = get_package_share_directory("multi_robot_mission_stack")
    p32 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mission_stack_pkg_dir, "launch", "p3_2_dual_advisory_visibility.launch.py")
        )
    )

    seam = Node(
        package="multi_robot_mission_stack",
        executable="spatial-advisory-tf-observer-r1",
        name="spatial_advisory_tf_observer_r1",
        output="screen",
        parameters=[
            {
                "robot_a_tf_topic": "/robot1_ns/tf",
                "robot_b_tf_topic": "/robot2_ns/tf",
                "robot_base_child_frame": "base_footprint",
                "distance_threshold_m": 2.5,
                "publish_topic": "/semantic/degraded_passage_p1_1",
                "source_robot_id": "robot1",
                "degradation_class": "narrow_clearance",
                "sensor_class": "tf_relative_pose",
            }
        ],
    )

    return LaunchDescription([p32, seam])

