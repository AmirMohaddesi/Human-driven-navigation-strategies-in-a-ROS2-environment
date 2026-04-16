"""
P3.2 — read-only terminal latest-state board for dual advisory ``std_msgs/String`` transports.

Subscribe only. No service/action clients. No control publishers.
"""

from __future__ import annotations

import json
import sys
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Must match ``mission_bridge_headless_dual_advisory_p11.launch.py`` transport names.
TOPIC_BLOCKED = "/semantic/blocked_passage_p1_1"
TOPIC_DEGRADED = "/semantic/degraded_passage_p1_1"
BOARD_PERIOD_SEC = 1.5
RECENT_EVENTS_MAX = 8


def _utc_wall_stamp() -> str:
    dt = datetime.now(timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S.") + f"{dt.microsecond // 1000:03d}Z"


@dataclass
class TopicStats:
    valid: int = 0
    malformed: int = 0
    empty: int = 0
    non_object: int = 0


@dataclass
class TopicLatest:
    recv: str = "-"
    fact_type: str = "-"
    location_ref: str = "-"
    belief_id: str = "-"
    raw_len: int = 0
    status: str = "NO_DATA"


class P3DualAdvisoryBoardNode(Node):
    def __init__(self) -> None:
        super().__init__("p3_2_dual_advisory_board")
        self._stats: dict[str, TopicStats] = {
            TOPIC_BLOCKED: TopicStats(),
            TOPIC_DEGRADED: TopicStats(),
        }
        self._latest: dict[str, TopicLatest] = {
            TOPIC_BLOCKED: TopicLatest(),
            TOPIC_DEGRADED: TopicLatest(),
        }
        self._recent_events: deque[str] = deque(maxlen=RECENT_EVENTS_MAX)
        self.create_subscription(String, TOPIC_BLOCKED, self._on_blocked, 10)
        self.create_subscription(String, TOPIC_DEGRADED, self._on_degraded, 10)
        self.create_timer(BOARD_PERIOD_SEC, self._render_board)
        self.get_logger().info(
            "P3.2 visibility-only board: subscribed to %s and %s (no control path)." % (TOPIC_BLOCKED, TOPIC_DEGRADED)
        )
        self._render_board()

    def _record_event(self, topic: str, status: str, detail: str) -> None:
        recv = _utc_wall_stamp()
        self._recent_events.appendleft("[P3.2_EVT] recv=%s topic=%s status=%s %s" % (recv, topic, status, detail))

    def _handle_payload(self, topic: str, data: str) -> None:
        recv = _utc_wall_stamp()
        stats = self._stats[topic]
        latest = self._latest[topic]
        latest.recv = recv
        latest.raw_len = len(data)

        if not data:
            stats.empty += 1
            latest.status = "EMPTY_PAYLOAD"
            latest.fact_type = "-"
            latest.location_ref = "-"
            latest.belief_id = "-"
            self._record_event(topic, latest.status, "raw_len=0")
            return

        try:
            obj: Any = json.loads(data)
        except json.JSONDecodeError as e:
            stats.malformed += 1
            latest.status = "UNPARSEABLE_JSON"
            latest.fact_type = "-"
            latest.location_ref = "-"
            latest.belief_id = "-"
            head = data.replace("\n", " ")
            if len(head) > 100:
                head = head[:100] + "..."
            self._record_event(topic, latest.status, "err=%r raw_head=%r" % (str(e), head))
            return

        if not isinstance(obj, dict):
            stats.non_object += 1
            latest.status = "JSON_NOT_OBJECT"
            latest.fact_type = type(obj).__name__
            latest.location_ref = "-"
            latest.belief_id = "-"
            self._record_event(topic, latest.status, "json_type=%s" % type(obj).__name__)
            return

        stats.valid += 1
        latest.status = "VALID"
        latest.fact_type = str(obj.get("fact_type", "?"))
        latest.location_ref = str(obj.get("location_ref", "?"))
        latest.belief_id = str(obj.get("belief_id", "?"))
        self._record_event(
            topic,
            latest.status,
            "fact_type=%r location_ref=%r belief_id=%r" % (latest.fact_type, latest.location_ref, latest.belief_id),
        )

    def _render_row(self, label: str, topic: str) -> str:
        latest = self._latest[topic]
        stats = self._stats[topic]
        return (
            "[P3.2_ROW] lane=%s topic=%s last_recv=%s status=%s fact_type=%r location_ref=%r belief_id=%r "
            "raw_len=%d counts(valid=%d malformed=%d empty=%d non_object=%d)"
            % (
                label,
                topic,
                latest.recv,
                latest.status,
                latest.fact_type,
                latest.location_ref,
                latest.belief_id,
                latest.raw_len,
                stats.valid,
                stats.malformed,
                stats.empty,
                stats.non_object,
            )
        )

    def _render_board(self) -> None:
        print("[P3.2_BOARD] ---- latest-state dual-advisory board ----", flush=True)
        print(self._render_row("blocked", TOPIC_BLOCKED), flush=True)
        print(self._render_row("degraded", TOPIC_DEGRADED), flush=True)
        print("[P3.2_RECENT] showing up to %d newest events" % RECENT_EVENTS_MAX, flush=True)
        if not self._recent_events:
            print("[P3.2_EVT] none yet", flush=True)
        else:
            for event in self._recent_events:
                print(event, flush=True)

    def _on_blocked(self, msg: String) -> None:
        self._handle_payload(TOPIC_BLOCKED, msg.data)

    def _on_degraded(self, msg: String) -> None:
        self._handle_payload(TOPIC_DEGRADED, msg.data)


def main() -> int:
    rclpy.init(args=sys.argv)
    node = P3DualAdvisoryBoardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
