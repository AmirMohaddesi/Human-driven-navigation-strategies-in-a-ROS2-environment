"""
V3.9 — two-node launch: semantic handoff service (optional transport mirror) + advisory witness.

Default domain: handoff and witness share the same DDS graph; witness subscribes to the frozen
V3.0.1 JSON topic so a separate process can hold the advisory store/facade.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    allow = ["robot1"]
    topic = "/semantic/blocked_passage_v301"

    handoff = Node(
        package="multi_robot_mission_stack",
        executable="semantic-production-handoff-v35",
        name="semantic_production_handoff_v35",
        output="screen",
        parameters=[
            {
                "allowed_source_robot_ids": allow,
                "mirror_ingested_record_to_transport": True,
                "mirror_transport_topic": topic,
            }
        ],
    )
    witness = Node(
        package="multi_robot_mission_stack",
        executable="advisory-seam-witness-v39",
        name="advisory_seam_witness_v39",
        output="screen",
        parameters=[
            {
                "transport_topic": topic,
                "allowed_source_robot_ids": allow,
                "service_name": "query_advisory_named_nav_v39",
            }
        ],
    )
    return LaunchDescription([handoff, witness])
