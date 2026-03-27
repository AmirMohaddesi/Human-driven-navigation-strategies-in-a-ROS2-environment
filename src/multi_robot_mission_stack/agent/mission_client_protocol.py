"""Abstract mission client boundary for LangGraph and tooling (no ROS imports)."""

from typing import Protocol, runtime_checkable


@runtime_checkable
class MissionClientProtocol(Protocol):
    """Contract for mission bridge operations; real ROS client implements this later."""

    def navigate_to_pose(self, robot_id: str, x: float, y: float, yaw: float) -> dict:
        ...

    def navigate_to_named_location(self, robot_id: str, location_name: str) -> dict:
        ...

    def get_navigation_state(self, robot_id: str, goal_id: str) -> dict:
        ...

    def cancel_navigation(self, robot_id: str, goal_id: str) -> dict:
        ...

    def close(self) -> None:
        ...
