"""
V4.2 — headless launch: mission bridge with V4.1 advisory transport enabled, Nav2 wait skipped.

Use for bounded advisory seam proof without a live Nav2 stack. Production stacks should keep
``wait_for_nav2_action_servers`` default true (omit this launch or override).
"""

from launch import LaunchDescription
from launch_ros.actions import Node

# Isolated topic so tests do not cross-talk with other demos.
ADVISORY_TOPIC = "/semantic/blocked_passage_v42_launch"


def generate_launch_description() -> LaunchDescription:
    bridge = Node(
        package="multi_robot_mission_stack",
        executable="mission_bridge_node",
        name="mission_bridge_node",
        output="screen",
        parameters=[
            {
                "wait_for_nav2_action_servers": False,
                "advisory_blocked_passage_transport_topic": ADVISORY_TOPIC,
                "advisory_blocked_passage_allowed_source_robot_ids": ["robot1"],
            }
        ],
    )
    return LaunchDescription([bridge])
