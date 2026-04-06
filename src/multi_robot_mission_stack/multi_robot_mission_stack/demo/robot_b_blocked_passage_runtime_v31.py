"""
V3.1 robot-B demo runtime: one shared ``BlockedPassageBeliefStore``, ROS JSON ingest subscriber,
and the existing facade/tools named-location policy (explicit ``now_utc`` on ``handle_command``).

Reuses frozen V3.0.1 components only; no transport redesign.
"""

from __future__ import annotations

from datetime import datetime
from typing import FrozenSet, Optional, TYPE_CHECKING

from ..agent.blocked_passage_shared_runtime_v301 import BlockedPassageSharedStoreRuntime
from ..agent.mission_policy import MissionPolicy
from ..agent.mission_client_protocol import MissionClientProtocol
from ..semantic.blocked_passage_v301 import BlockedPassageBeliefStore, IngestResult
from ..transport.blocked_passage_receiver_node import BlockedPassageTransportReceiverNode

if TYPE_CHECKING:
    from ..client.mission_client import MissionClient


def _allowlist_from_param_list(values: list) -> Optional[FrozenSet[str]]:
    if not values:
        return None
    out = frozenset(str(x).strip() for x in values if str(x).strip())
    return out if out else None


class RobotBBlockedPassageDemoRuntime:
    """
    Dedicated robot-B shell: ``BlockedPassageSharedStoreRuntime`` + factory for transport receiver
    using the **same** store instance as ``MissionTools`` / ``facade``.
    """

    def __init__(
        self,
        mission_client: MissionClientProtocol,
        *,
        policy: Optional[MissionPolicy] = None,
        allowed_source_robot_ids: Optional[FrozenSet[str]] = None,
    ) -> None:
        self._core = BlockedPassageSharedStoreRuntime(
            mission_client,
            policy=policy,
            allowed_source_robot_ids=allowed_source_robot_ids,
        )

    @property
    def store(self) -> BlockedPassageBeliefStore:
        return self._core.store

    @property
    def core(self) -> BlockedPassageSharedStoreRuntime:
        """Underlying V3.0.1 composition (tests / advanced wiring)."""
        return self._core

    @property
    def facade(self):
        return self._core.facade

    def ingest_transport_payload(self, payload: str, *, now_utc: datetime) -> IngestResult:
        return self._core.ingest_transport_payload(payload, now_utc=now_utc)

    def create_transport_receiver_node(self) -> BlockedPassageTransportReceiverNode:
        """ROS subscriber on the default V3.0.1 topic; ingests into ``self.store``."""
        return BlockedPassageTransportReceiverNode(store=self.store)


def main() -> None:
    """Spin transport receiver + mission client node (bridge calls use ``spin_until_future_complete``)."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    from ..client.mission_client import MissionClient

    rclpy.init()
    param_node = None
    recv_node = None
    client: Optional[MissionClient] = None
    try:
        param_node = rclpy.create_node("robot_b_blocked_passage_demo_bootstrap")
        param_node.declare_parameter("allowed_source_robot_ids", [])
        param_node.declare_parameter("bridge_node_name", "mission_bridge_node")
        raw_allow = list(param_node.get_parameter("allowed_source_robot_ids").value)
        bridge = str(param_node.get_parameter("bridge_node_name").value or "mission_bridge_node")
        allowed = _allowlist_from_param_list(raw_allow)

        param_node.destroy_node()
        param_node = None

        client = MissionClient(bridge_node_name=bridge)
        runtime = RobotBBlockedPassageDemoRuntime(
            client,
            allowed_source_robot_ids=allowed,
        )
        recv_node = runtime.create_transport_receiver_node()
        recv_node.get_logger().info(
            "robot B blocked_passage demo: shared store + subscriber; "
            "use facade.handle_command(..., now_utc=...) with explicit now_utc from caller/tooling"
        )

        executor = MultiThreadedExecutor()
        executor.add_node(recv_node)
        if getattr(client, "_node", None) is not None:
            executor.add_node(client._node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if recv_node is not None:
            try:
                recv_node.destroy_node()
            except Exception:
                pass
        if client is not None:
            client.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
