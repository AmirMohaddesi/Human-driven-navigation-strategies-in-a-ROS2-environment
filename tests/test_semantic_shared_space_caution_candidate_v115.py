"""V11.5 local candidate-to-record assembly for shared_space_caution."""

from __future__ import annotations

import uuid
from datetime import datetime, timezone

from multi_robot_mission_stack.semantic.shared_space_caution_candidate_v115 import (
    CANDIDATE_INVALID,
    FINAL_SCHEMA_REJECT,
    SharedSpaceCautionAssembledAccept,
    SharedSpaceCautionAssembledReject,
    SharedSpaceCautionAssemblyCandidate,
    assemble_shared_space_caution_record_from_candidate,
)
from multi_robot_mission_stack.semantic.shared_space_caution_v111 import (
    validate_shared_space_caution_record,
)


def _valid_candidate() -> SharedSpaceCautionAssemblyCandidate:
    return SharedSpaceCautionAssemblyCandidate(
        location_ref="base",
        source_robot_id="robot1",
        confidence=0.7,
        ttl_sec=120.0,
        caution_class="pedestrian_activity",
        sensor_class="pedestrian_proxy",
    )


def test_v115_valid_candidate_assembles_to_validator_ok_record() -> None:
    ts = datetime(2026, 4, 8, 12, 30, 0, tzinfo=timezone.utc)
    out = assemble_shared_space_caution_record_from_candidate(
        _valid_candidate(),
        timestamp_utc=ts,
        belief_id="123e4567-e89b-42d3-a456-426614174000",
        observation_id="223e4567-e89b-42d3-a456-426614174111",
    )
    assert isinstance(out, SharedSpaceCautionAssembledAccept)
    ok, errs = validate_shared_space_caution_record(out.record)
    assert ok and errs == []
    assert out.record["schema_version"] == "v11.1"
    assert out.record["fact_type"] == "shared_space_caution"
    assert out.record["verification_status"] == "unverified"
    assert out.record["timestamp_utc"] == "2026-04-08T12:30:00Z"


def test_v115_invalid_candidate_rejected_before_assembly() -> None:
    bad = SharedSpaceCautionAssemblyCandidate(
        location_ref="base",
        source_robot_id="robot1",
        confidence=0.7,
        ttl_sec=120.0,
        caution_class="slow_zone",
        sensor_class="pedestrian_proxy",
    )
    ts = datetime(2026, 4, 8, 12, 30, 0, tzinfo=timezone.utc)
    out = assemble_shared_space_caution_record_from_candidate(bad, timestamp_utc=ts)
    assert isinstance(out, SharedSpaceCautionAssembledReject)
    assert out.reason == CANDIDATE_INVALID
    assert "caution_class" in out.detail


def test_v115_final_validator_rejects_invalid_explicit_belief_id() -> None:
    ts = datetime(2026, 4, 8, 12, 30, 0, tzinfo=timezone.utc)
    bad_bid = str(uuid.uuid3(uuid.NAMESPACE_DNS, "not-v4"))
    out = assemble_shared_space_caution_record_from_candidate(
        _valid_candidate(),
        timestamp_utc=ts,
        belief_id=bad_bid,
        observation_id="223e4567-e89b-42d3-a456-426614174111",
    )
    assert isinstance(out, SharedSpaceCautionAssembledReject)
    assert out.reason == FINAL_SCHEMA_REJECT
    assert "belief_id" in out.detail.lower() or "belief" in out.detail.lower()


def test_v115_omitted_ids_generated_and_record_valid() -> None:
    ts = datetime(2026, 4, 8, 12, 30, 0, tzinfo=timezone.utc)
    out = assemble_shared_space_caution_record_from_candidate(_valid_candidate(), timestamp_utc=ts)
    assert isinstance(out, SharedSpaceCautionAssembledAccept)
    ok, _ = validate_shared_space_caution_record(out.record)
    assert ok
    assert len(str(out.record["belief_id"])) == 36
