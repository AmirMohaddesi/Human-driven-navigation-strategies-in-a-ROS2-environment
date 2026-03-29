"""
External-facing command shapes (API / operators / upstream systems).

These are **not** the internal request format consumed by ``MissionGraph``.
Use ``CommandAdapter`` to translate into graph requests:

* ``action``: ``"navigate_to_named_location"`` | ``"get_navigation_state"``
* plus ``robot_id`` and action-specific fields.
"""

from __future__ import annotations

from typing import Literal, TypedDict, Union


class NavigateToNamedLocationCommand(TypedDict):
    """External navigate command: send a robot to a named location."""

    type: Literal["navigate"]
    target: Literal["named_location"]
    robot_id: str
    location_name: str


class GetNavigationStateCommand(TypedDict):
    """External query command: read navigation state for a goal."""

    type: Literal["query"]
    target: Literal["navigation_state"]
    robot_id: str
    goal_id: str


ExternalMissionCommand = Union[
    NavigateToNamedLocationCommand,
    GetNavigationStateCommand,
]
