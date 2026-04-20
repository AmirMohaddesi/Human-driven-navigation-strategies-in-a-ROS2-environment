"""
R1 showcase helper: runtime TF seam + P3.2 board + R2 marker + R3 local hold (no second mission_bridge_node).

Use alongside ``runtime_stack`` / ``fully_integrated_swarm`` where robot TF is available.

For a **single-process** R1/R2/R3 demo (bridge + board + R1 + R2 marker + R3 hold, one bridge), use::

    ros2 launch multi_robot_mission_stack runtime_stack.launch.py profile:=sim_nav_bridge_r1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    mission_stack_pkg_dir = get_package_share_directory("multi_robot_mission_stack")
    p32_board_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mission_stack_pkg_dir, "launch", "p3_2_dual_advisory_visibility.launch.py")
        ),
        launch_arguments={"launch_bridge": "false"}.items(),
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
    r2_marker = Node(
        package="multi_robot_mission_stack",
        executable="r2-degraded-visual-marker",
        name="r2_degraded_visual_marker",
        output="screen",
    )
    r3_hold = Node(
        package="multi_robot_mission_stack",
        executable="r3-local-hold-from-r1",
        name="r3_local_hold_from_r1",
        output="screen",
    )

    return LaunchDescription([p32_board_only, seam, r2_marker, r3_hold])

