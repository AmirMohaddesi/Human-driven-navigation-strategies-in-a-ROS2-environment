"""
P3.2 — P1.1 headless dual-advisory bridge + read-only terminal latest-state board.

Board node subscribes only; it does not publish control or call services.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

BLOCKED_TOPIC = "/semantic/blocked_passage_p1_1"
DEGRADED_TOPIC = "/semantic/degraded_passage_p1_1"


def generate_launch_description() -> LaunchDescription:
    bridge = Node(
        package="multi_robot_mission_stack",
        executable="mission_bridge_node",
        name="mission_bridge_node",
        output="screen",
        parameters=[
            {
                "wait_for_nav2_action_servers": False,
                "advisory_blocked_passage_transport_topic": BLOCKED_TOPIC,
                "advisory_blocked_passage_allowed_source_robot_ids": ["robot1"],
                "advisory_degraded_passage_transport_topic": DEGRADED_TOPIC,
                "advisory_degraded_passage_allowed_source_robot_ids": ["robot1"],
            }
        ],
    )
    board = Node(
        package="multi_robot_mission_stack",
        executable="p3_2_dual_advisory_board",
        name="p3_2_dual_advisory_board",
        output="screen",
    )
    return LaunchDescription([bridge, board])
