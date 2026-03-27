"""Typed state carried through the mission LangGraph."""

from __future__ import annotations

from typing import TypedDict


class MissionGraphState(TypedDict, total=False):
    """
    Graph state for MissionGraph.

    ``request`` — structured mission input (see below). Set by ``invoke()`` as the
    only required initial field.

    ``action`` — non-empty string taken from ``request["action"]`` after parse;
    empty when parse failed.

    ``result`` — normalized tool outcome or structured failure; set by terminal nodes.

    ``error`` — human-readable parse/validation message when the request shape is
    invalid; empty string when parse succeeded.

    ``policy_allowed`` — set after ``policy_check``; ``True`` when ``MissionPolicy``
    allows the request.

    ``policy_message`` — message from ``MissionPolicy.evaluate`` (allow or deny text).

    Expected ``request`` shape (explicit keys; extra keys are ignored):

    * ``action`` (str, required): one of ``"navigate_to_named_location"``,
      ``"get_navigation_state"``.

    * For ``navigate_to_named_location``:
        - ``robot_id`` (str, required)
        - ``location_name`` (str, required)

    * For ``get_navigation_state``:
        - ``robot_id`` (str, required)
        - ``goal_id`` (str, required)

    Examples::

        {
            "action": "navigate_to_named_location",
            "robot_id": "robot1",
            "location_name": "base",
        }

        {
            "action": "get_navigation_state",
            "robot_id": "robot1",
            "goal_id": "mock-goal-001",
        }
    """

    request: dict
    action: str
    result: dict
    error: str
    policy_allowed: bool
    policy_message: str
