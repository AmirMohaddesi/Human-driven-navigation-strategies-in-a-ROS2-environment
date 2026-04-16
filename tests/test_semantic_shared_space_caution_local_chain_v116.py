"""V11.6 — deterministic local proof chain: assembly → ingest → has_active → advisory (no ROS)."""

from __future__ import annotations

from datetime import datetime, timezone

from multi_robot_mission_stack.semantic.shared_space_caution_candidate_v115 import (
    SharedSpaceCautionAssembledAccept,
    SharedSpaceCautionAssemblyCandidate,
    assemble_shared_space_caution_record_from_candidate,
)
from multi_robot_mission_stack.semantic.shared_space_caution_v111 import (
    ADVISORY_QUERY_REPORT_SCHEMA_VERSION,
    FACT_TYPE_SHARED_SPACE_CAUTION,
    SharedSpaceCautionBeliefStore,
)


def _fixed_now() -> datetime:
    return datetime(2026, 4, 8, 12, 30, 30, tzinfo=timezone.utc)


def test_v116_assemble_ingest_has_active_advisory_chain() -> None:
    now = _fixed_now()
    bid = "123e4567-e89b-42d3-a456-426614174000"
    oid = "223e4567-e89b-42d3-a456-426614174111"
    cand = SharedSpaceCautionAssemblyCandidate(
        location_ref="base",
        source_robot_id="robot1",
        confidence=0.7,
        ttl_sec=120.0,
        caution_class="pedestrian_activity",
        sensor_class="pedestrian_proxy",
    )
    out = assemble_shared_space_caution_record_from_candidate(
        cand,
        timestamp_utc=now,
        belief_id=bid,
        observation_id=oid,
    )
    assert isinstance(out, SharedSpaceCautionAssembledAccept)
    rec = out.record
    assert rec["belief_id"] == bid
    assert rec["location_ref"] == "base"

    store = SharedSpaceCautionBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    ing = store.ingest(rec, now_utc=now)
    assert ing.stored and not ing.rejected and not ing.duplicate_ignored

    q = store.has_active_shared_space_caution("base", now_utc=now)
    assert q.has_active
    assert q.active_belief_ids == (bid,)

    rep = store.query_advisory_shared_space_caution("base", now_utc=now)
    assert rep.schema_version == ADVISORY_QUERY_REPORT_SCHEMA_VERSION
    assert rep.fact_type == FACT_TYPE_SHARED_SPACE_CAUTION
    assert rep.location_ref == "base"
    assert rep.has_active
    assert rep.active_belief_ids == (bid,)
    assert len(rep.entries) == 1
    e = rep.entries[0]
    assert e.belief_id == bid
    assert e.source_robot_id == "robot1"
    assert e.caution_class == "pedestrian_activity"
    assert e.confidence == 0.7
    assert e.timestamp_utc == "2026-04-08T12:30:30Z"


def test_v116_duplicate_reingest_same_assembled_record() -> None:
    now = _fixed_now()
    cand = SharedSpaceCautionAssemblyCandidate(
        location_ref="base",
        source_robot_id="robot1",
        confidence=0.7,
        ttl_sec=120.0,
        caution_class="pedestrian_activity",
        sensor_class="pedestrian_proxy",
    )
    out = assemble_shared_space_caution_record_from_candidate(
        cand,
        timestamp_utc=now,
        belief_id="323e4567-e89b-42d3-a456-426614174002",
        observation_id="423e4567-e89b-42d3-a456-426614174003",
    )
    assert isinstance(out, SharedSpaceCautionAssembledAccept)
    rec = out.record
    store = SharedSpaceCautionBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    assert store.ingest(rec, now_utc=now).stored
    dup = store.ingest(rec, now_utc=now)
    assert dup.duplicate_ignored and not dup.stored and not dup.rejected
    assert len(store) == 1

    rep = store.query_advisory_shared_space_caution("base", now_utc=now)
    assert rep.has_active
    assert len(rep.entries) == 1
    assert len(rep.active_belief_ids) == 1
