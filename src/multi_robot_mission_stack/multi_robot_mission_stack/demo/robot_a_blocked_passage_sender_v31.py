"""
V3.1 robot A: one-shot publish of frozen ``blocked_passage`` JSON on ``std_msgs/String``.

Reuses V3.0.1 stub + encode + validate; no transport redesign.
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime, timezone
from typing import Optional, Tuple

from ..semantic.blocked_passage_local_stub_v301 import build_blocked_passage_record_stub
from ..semantic.blocked_passage_v301 import validate_blocked_passage_record
from ..transport.blocked_passage_json_v301 import (
    TRANSPORT_TOPIC_DEFAULT,
    decode_blocked_passage_transport_payload,
    encode_blocked_passage_record_json,
)


def _parse_timestamp_utc_arg(s: str) -> datetime:
    raw = str(s).strip()
    if raw.endswith("Z"):
        raw = raw[:-1] + "+00:00"
    dt = datetime.fromisoformat(raw)
    if dt.tzinfo is None:
        raise ValueError("timestamp_utc must be timezone-aware (RFC 3339 with Z or offset)")
    return dt.astimezone(timezone.utc)


def build_robot_a_blocked_passage_wire(
    *,
    belief_id: str,
    source_robot_id: str,
    location_ref: str,
    confidence: float,
    timestamp_utc: datetime,
    ttl_sec: float,
    sensor_class: str,
    observation_id: str,
) -> str:
    """Build canonical compact JSON wire string from explicit stub inputs (schema-valid)."""
    rec = build_blocked_passage_record_stub(
        belief_id=belief_id,
        source_robot_id=source_robot_id,
        location_ref=location_ref,
        confidence=confidence,
        timestamp_utc=timestamp_utc,
        ttl_sec=ttl_sec,
        sensor_class=sensor_class,
        observation_id=observation_id,
    )
    return encode_blocked_passage_record_json(rec)


def prepare_robot_a_wire_from_payload_json(payload_json: str) -> str:
    """
    Parse full JSON object, validate frozen schema, return canonical compact JSON string.
    Raises ``ValueError`` on empty, invalid JSON, or schema failure.
    """
    if not str(payload_json).strip():
        raise ValueError("payload_json is empty")
    record = decode_blocked_passage_transport_payload(payload_json)
    ok, errs = validate_blocked_passage_record(record)
    if not ok:
        raise ValueError("; ".join(errs))
    return encode_blocked_passage_record_json(record)


def resolve_robot_a_wire_for_publish(
    *,
    payload_json: str = "",
    belief_id: str = "",
    source_robot_id: str = "",
    location_ref: str = "",
    confidence: Optional[float] = None,
    timestamp_utc_str: str = "",
    ttl_sec: Optional[float] = None,
    sensor_class: str = "",
    observation_id: str = "",
) -> Tuple[str, Optional[str]]:
    """
    Return ``(wire, error)``. ``error`` is a human string if publishing should be skipped.
    """
    if str(payload_json).strip():
        try:
            return prepare_robot_a_wire_from_payload_json(payload_json), None
        except ValueError as exc:
            return "", str(exc)

    missing = []
    if not str(belief_id).strip():
        missing.append("belief_id")
    if not str(source_robot_id).strip():
        missing.append("source_robot_id")
    if not str(location_ref).strip():
        missing.append("location_ref")
    if confidence is None:
        missing.append("confidence")
    if not str(timestamp_utc_str).strip():
        missing.append("timestamp_utc")
    if ttl_sec is None:
        missing.append("ttl_sec")
    if not str(sensor_class).strip():
        missing.append("sensor_class")
    if not str(observation_id).strip():
        missing.append("observation_id")
    if missing:
        return "", "missing required fields for stub mode: " + ", ".join(missing)

    try:
        ts = _parse_timestamp_utc_arg(timestamp_utc_str)
    except (ValueError, TypeError) as exc:
        return "", f"timestamp_utc: {exc}"

    try:
        wire = build_robot_a_blocked_passage_wire(
            belief_id=belief_id.strip(),
            source_robot_id=source_robot_id.strip(),
            location_ref=location_ref.strip(),
            confidence=float(confidence),
            timestamp_utc=ts,
            ttl_sec=float(ttl_sec),
            sensor_class=sensor_class.strip(),
            observation_id=observation_id.strip(),
        )
    except ValueError as exc:
        return "", str(exc)
    return wire, None


def publish_robot_a_blocked_passage_once(
    wire: str,
    *,
    topic: str = TRANSPORT_TOPIC_DEFAULT,
) -> None:
    """``rclpy`` one-shot publish; caller must not hold a conflicting context."""
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = None
    try:
        node = rclpy.create_node("robot_a_blocked_passage_sender_once")
        pub = node.create_publisher(String, topic, 10)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
        msg = String()
        msg.data = wire
        pub.publish(msg)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Robot A: publish one blocked_passage JSON (std_msgs/String) then exit.",
    )
    p.add_argument("--topic", default=TRANSPORT_TOPIC_DEFAULT, help="ROS topic name")
    p.add_argument(
        "--payload-json",
        default="",
        help="Full JSON object for the record (validated); if set, stub flags are ignored",
    )
    p.add_argument("--belief-id", default="")
    p.add_argument("--source-robot-id", default="")
    p.add_argument("--location-ref", default="")
    p.add_argument("--confidence", type=float, default=None)
    p.add_argument(
        "--timestamp-utc",
        default="",
        help="RFC 3339 instant, e.g. 2026-04-02T18:30:00.000Z",
    )
    p.add_argument("--ttl-sec", type=float, default=None)
    p.add_argument("--sensor-class", default="")
    p.add_argument("--observation-id", default="")
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Print wire to stdout and exit without ROS publish",
    )
    return p


def main(argv: Optional[list] = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    wire, err = resolve_robot_a_wire_for_publish(
        payload_json=args.payload_json,
        belief_id=args.belief_id,
        source_robot_id=args.source_robot_id,
        location_ref=args.location_ref,
        confidence=args.confidence,
        timestamp_utc_str=args.timestamp_utc,
        ttl_sec=args.ttl_sec,
        sensor_class=args.sensor_class,
        observation_id=args.observation_id,
    )
    if err:
        print(err, file=sys.stderr)
        return 1
    if args.dry_run:
        print(wire)
        return 0
    publish_robot_a_blocked_passage_once(wire, topic=str(args.topic or TRANSPORT_TOPIC_DEFAULT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
