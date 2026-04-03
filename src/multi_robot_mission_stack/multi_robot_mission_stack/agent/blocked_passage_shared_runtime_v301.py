"""
V3.0.1 single-process composition: one ``BlockedPassageBeliefStore`` shared by transport ingest
and ``MissionTools`` (named-location policy). No second state model.
"""

from __future__ import annotations

from datetime import datetime
from typing import FrozenSet, Optional

from ..semantic.blocked_passage_v301 import BlockedPassageBeliefStore, IngestResult
from ..transport.blocked_passage_json_v301 import ingest_blocked_passage_transport_payload
from .command_adapter import CommandAdapter
from .mission_agent_facade import MissionAgentFacade
from .mission_client_protocol import MissionClientProtocol
from .mission_graph import MissionGraph
from .mission_policy import MissionPolicy
from .mission_tools import MissionTools
from .policy_config import build_default_policy_config


class BlockedPassageSharedStoreRuntime:
    """
    Constructs mission facade/tools on the same ``BlockedPassageBeliefStore`` used for
    ``ingest_transport_payload`` (JSON wire -> ``store.ingest``).
    """

    def __init__(
        self,
        mission_client: MissionClientProtocol,
        *,
        policy: Optional[MissionPolicy] = None,
        allowed_source_robot_ids: Optional[FrozenSet[str]] = None,
        store: Optional[BlockedPassageBeliefStore] = None,
    ) -> None:
        if store is not None:
            self.store = store
        else:
            self.store = BlockedPassageBeliefStore(
                allowed_source_robot_ids=allowed_source_robot_ids,
            )
        resolved_policy = policy or MissionPolicy(build_default_policy_config())
        self._tools = MissionTools(
            mission_client,
            blocked_passage_store=self.store,
        )
        self._graph = MissionGraph(self._tools, policy=resolved_policy)
        self._adapter = CommandAdapter()
        self.facade = MissionAgentFacade(
            self._adapter,
            self._graph,
            mission_client=mission_client,
        )

    def ingest_transport_payload(self, payload: str, *, now_utc: datetime) -> IngestResult:
        """Same ingest path as the ROS receiver callback (decode JSON -> ``store.ingest``)."""
        return ingest_blocked_passage_transport_payload(self.store, payload, now_utc=now_utc)
