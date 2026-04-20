"""
Composable multi-robot runtime: Gazebo + optional Nav2 / SLAM / bridge / merge / RViz / R1 / P3 board / R2 marker / R3 local hold.

Select a ``profile`` for opinionated defaults, then override individual layers with ``enable_*:=true|false``
when you need fine control (omit value or use ``auto`` to follow the profile).

Canonical entrypoint for new work; ``fully_integrated_swarm.launch.py`` remains an alias for ``profile:=full_demo``.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from multi_robot_mission_stack.launch_support.stack_composition import generate_profiled_stack


def generate_launch_description() -> LaunchDescription:
    # Keep TurtleBot3 model convention aligned with historical launches.
    _ = os.environ.get("TURTLEBOT3_MODEL", "waffle")

    profile_choices = (
        "sim_core | sim_nav | sim_nav_slam | sim_nav_bridge | sim_nav_bridge_r1 | "
        "debug_visibility | full_demo"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="sim_nav_bridge",
                description=(
                    "Stack profile (defaults for all enable_* layers). Options: " + profile_choices
                ),
            ),
            DeclareLaunchArgument(
                "robot_count",
                default_value="2",
                description="How many robots to spawn (robot1 … robotN). Bridge IDs remain robot1/robot2 for N>=2.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Forwarded to nodes that accept use_sim_time.",
            ),
            DeclareLaunchArgument(
                "gazebo_software_gl",
                default_value="false",
                description="If true, forces software GL for Gazebo (debug).",
            ),
            DeclareLaunchArgument(
                "enable_nav",
                default_value="auto",
                description="Nav2 localization_navigation include per robot: true | false | auto (follow profile).",
            ),
            DeclareLaunchArgument(
                "enable_slam",
                default_value="auto",
                description="SLAM Toolbox per robot: true | false | auto.",
            ),
            DeclareLaunchArgument(
                "enable_per_robot_rviz",
                default_value="auto",
                description="Per-robot Nav2 RViz: true | false | auto.",
            ),
            DeclareLaunchArgument(
                "enable_merge_map",
                default_value="auto",
                description="Global merge_map_node: true | false | auto.",
            ),
            DeclareLaunchArgument(
                "enable_merge_rviz",
                default_value="auto",
                description="Merge-map RViz: true | false | auto.",
            ),
            DeclareLaunchArgument(
                "enable_mission_bridge",
                default_value="auto",
                description="mission_bridge_node (single instance): true | false | auto.",
            ),
            DeclareLaunchArgument(
                "enable_bridge_advisory_transport",
                default_value="auto",
                description=(
                    "P3 advisory transport params on bridge (blocked/degraded topics): true | false | auto."
                ),
            ),
            DeclareLaunchArgument(
                "enable_p3_board",
                default_value="auto",
                description="P3.2 read-only advisory board: true | false | auto.",
            ),
            DeclareLaunchArgument(
                "enable_r1_seam",
                default_value="auto",
                description="R1 TF-distance degraded advisory publisher: true | false | auto.",
            ),
            DeclareLaunchArgument(
                "enable_r2_visual_consequence",
                default_value="auto",
                description="R2 bounded visible consequence marker from R1 degraded advisories: true | false | auto.",
            ),
            DeclareLaunchArgument(
                "enable_r3_local_hold",
                default_value="auto",
                description="R3 bounded local robot hold from R1 degraded advisories: true | false | auto.",
            ),
            OpaqueFunction(function=generate_profiled_stack),
        ]
    )
