"""Single entry point for external callers: adapter + graph orchestration."""

from __future__ import annotations

from typing import Any, Dict, Optional

from .command_adapter import CommandAdapter
from .mission_client_protocol import MissionClientProtocol
from .mission_graph import MissionGraph
from .mission_policy import MissionPolicy
from .mission_tools import MissionTools
from .mock_mission_client import MockMissionClient
from .policy_config import build_default_policy_config


class MissionAgentFacade:
    """
    Orchestrates CommandAdapter then MissionGraph.

    Optional ``mission_client`` is stored only so ``close()`` can release
    protocol implementations that own resources (e.g. future ROS clients).
    """

    def __init__(
        self,
        adapter: CommandAdapter,
        graph: MissionGraph,
        *,
        mission_client: Optional[MissionClientProtocol] = None,
    ) -> None:
        self._adapter = adapter
        self._graph = graph
        self._mission_client = mission_client

    @classmethod
    def with_mock(cls, policy: Optional[MissionPolicy] = None) -> MissionAgentFacade:
        """Build a facade backed by ``MockMissionClient`` (no ROS)."""
        client = MockMissionClient()
        tools = MissionTools(client)
        resolved_policy = (
            policy if policy is not None else MissionPolicy(build_default_policy_config())
        )
        graph = MissionGraph(tools, policy=resolved_policy)
        adapter = CommandAdapter()
        return cls(adapter, graph, mission_client=client)

    @classmethod
    def with_ros(
        cls,
        policy: Optional[MissionPolicy] = None,
        bridge_node_name: str = "mission_bridge_node",
    ) -> MissionAgentFacade:
        """Build a facade backed by ``MissionClient`` (mission bridge ROS services)."""
        from ..client.mission_client import MissionClient

        client = MissionClient(bridge_node_name=bridge_node_name)
        tools = MissionTools(client)
        resolved_policy = (
            policy if policy is not None else MissionPolicy(build_default_policy_config())
        )
        graph = MissionGraph(tools, policy=resolved_policy)
        adapter = CommandAdapter()
        return cls(adapter, graph, mission_client=client)

    @classmethod
    def from_components(
        cls,
        *,
        adapter: CommandAdapter,
        graph: MissionGraph,
        mission_client: Optional[MissionClientProtocol] = None,
    ) -> MissionAgentFacade:
        """Assemble a facade from explicit adapter, graph, and optional client for ``close()``."""
        return cls(adapter, graph, mission_client=mission_client)

    def handle_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        adapted = self._adapter.adapt_and_validate(command)
        if adapted.get("status") == "failed":
            return adapted
        return self._graph.invoke(adapted)

    def close(self) -> None:
        client = self._mission_client
        if client is None:
            return
        try:
            client.close()
        except Exception:
            pass
