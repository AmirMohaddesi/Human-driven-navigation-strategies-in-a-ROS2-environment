"""V3.1 robot A sender: wire shape, schema, resolve errors (mostly ROS-free)."""

from __future__ import annotations

import uuid
from datetime import datetime, timezone

import pytest

from multi_robot_mission_stack.demo.robot_a_blocked_passage_sender_v31 import (
    build_robot_a_blocked_passage_wire,
    prepare_robot_a_wire_from_payload_json,
    resolve_robot_a_wire_for_publish,
)
from multi_robot_mission_stack.semantic.blocked_passage_v301 import (
    SCHEMA_VERSION,
    validate_blocked_passage_record,
)
from multi_robot_mission_stack.transport.blocked_passage_json_v301 import (
    decode_blocked_passage_transport_payload,
)


def _t0() -> datetime:
    return datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)


def _uuid4() -> str:
    return "a1b2c3d4-e5f6-4789-a012-3456789abcde"


def _obs() -> str:
    return "f47ac10b-58cc-4372-a567-0e02b2c3d479"


def test_explicit_stub_wire_matches_transport_encoding_and_validates() -> None:
    wire = build_robot_a_blocked_passage_wire(
        belief_id=_uuid4(),
        source_robot_id="robot_a",
        location_ref="base",
        confidence=0.85,
        timestamp_utc=_t0(),
        ttl_sec=120.0,
        sensor_class="demo_sender",
        observation_id=_obs(),
    )
    rec = decode_blocked_passage_transport_payload(wire)
    ok, errs = validate_blocked_passage_record(rec)
    assert ok and errs == []
    assert rec["schema_version"] == SCHEMA_VERSION
    assert rec["fact_type"] == "blocked_passage"
    assert "," in wire and wire.startswith("{")


def test_payload_json_mode_validates_and_canonicalizes() -> None:
    inner = (
        '{"belief_id":"%s","confidence":0.85,"fact_type":"blocked_passage",'
        '"location_ref":"base","provenance":{"observation_id":"%s","sensor_class":"x"},'
        '"schema_version":"v3.0.1","source_robot_id":"robot_a",'
        '"timestamp_utc":"2026-04-02T18:30:00.000Z","ttl_sec":120.0,'
        '"verification_status":"unverified"}'
        % (_uuid4(), _obs())
    )
    wire = prepare_robot_a_wire_from_payload_json(inner)
    rec = decode_blocked_passage_transport_payload(wire)
    assert validate_blocked_passage_record(rec)[0]


def test_prepare_empty_payload_raises() -> None:
    with pytest.raises(ValueError, match="empty"):
        prepare_robot_a_wire_from_payload_json("")
    with pytest.raises(ValueError, match="empty"):
        prepare_robot_a_wire_from_payload_json("   ")


def test_prepare_invalid_schema_json_raises() -> None:
    bad = '{"schema_version":"v9","belief_id":"%s"}' % _uuid4()
    with pytest.raises(ValueError):
        prepare_robot_a_wire_from_payload_json(bad)


def test_resolve_skips_publish_on_empty_stub_mode() -> None:
    wire, err = resolve_robot_a_wire_for_publish()
    assert wire == "" and err is not None and "missing required" in err.lower()


def test_resolve_payload_json_mode_invalid_returns_error_not_wire() -> None:
    wire, err = resolve_robot_a_wire_for_publish(payload_json="{not json")
    assert wire == "" and err is not None


def test_resolve_stub_mode_bad_belief_uuid_returns_error() -> None:
    wire, err = resolve_robot_a_wire_for_publish(
        belief_id=str(uuid.uuid3(uuid.NAMESPACE_DNS, "nv4")),
        source_robot_id="robot_a",
        location_ref="base",
        confidence=0.85,
        timestamp_utc_str="2026-04-02T18:30:00.000Z",
        ttl_sec=120.0,
        sensor_class="s",
        observation_id=_obs(),
    )
    assert wire == "" and err is not None and "belief_id" in err.lower()


def test_publish_once_does_not_crash_when_rclpy_available() -> None:
    rclpy = pytest.importorskip("rclpy")
    from multi_robot_mission_stack.demo.robot_a_blocked_passage_sender_v31 import (
        publish_robot_a_blocked_passage_once,
    )

    wire = build_robot_a_blocked_passage_wire(
        belief_id=_uuid4(),
        source_robot_id="robot_a",
        location_ref="base",
        confidence=0.85,
        timestamp_utc=_t0(),
        ttl_sec=120.0,
        sensor_class="demo_sender",
        observation_id=_obs(),
    )
    publish_robot_a_blocked_passage_once(wire)
