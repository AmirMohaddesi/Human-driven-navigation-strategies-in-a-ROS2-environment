"""
V4.3 — bounded end-to-end: semantic handoff (mirror on ingest) + mission bridge (advisory transport).

Single shared frozen V3.0.1 JSON topic connects handoff publish to bridge subscribe. No witness node;
bridge is the advisory consumer. Nav2 wait skipped on the bridge for headless proof.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

E2E_TOPIC = "/semantic/blocked_passage_v43_e2e"
ALLOW = ["robot1"]


def generate_launch_description() -> LaunchDescription:
    handoff = Node(
        package="multi_robot_mission_stack",
        executable="semantic-production-handoff-v35",
        name="semantic_production_handoff_v35",
        output="screen",
        parameters=[
            {
                "allowed_source_robot_ids": ALLOW,
                "mirror_ingested_record_to_transport": True,
                "mirror_transport_topic": E2E_TOPIC,
            }
        ],
    )
    bridge = Node(
        package="multi_robot_mission_stack",
        executable="mission_bridge_node",
        name="mission_bridge_node",
        output="screen",
        parameters=[
            {
                "wait_for_nav2_action_servers": False,
                "advisory_blocked_passage_transport_topic": E2E_TOPIC,
                "advisory_blocked_passage_allowed_source_robot_ids": ALLOW,
            }
        ],
    )
    return LaunchDescription([handoff, bridge])
