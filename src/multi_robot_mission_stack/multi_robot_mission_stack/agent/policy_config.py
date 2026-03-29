"""Mission policy allowlists (explicit configuration, no ROS)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet


@dataclass(frozen=True)
class MissionPolicyConfig:
    """Allowlists for mission requests evaluated before tool execution."""

    allowed_actions: FrozenSet[str]
    allowed_robot_ids: FrozenSet[str]
    allowed_locations: FrozenSet[str]


def build_default_policy_config() -> MissionPolicyConfig:
    """Development-friendly defaults aligned with the current MissionGraph actions."""
    return MissionPolicyConfig(
        allowed_actions=frozenset(
            {
                "navigate_to_named_location",
                "navigate_to_pose",
                "get_navigation_state",
                "cancel_navigation",
            }
        ),
        allowed_robot_ids=frozenset({"robot1", "robot2"}),
        allowed_locations=frozenset({"base", "test_goal"}),
    )
