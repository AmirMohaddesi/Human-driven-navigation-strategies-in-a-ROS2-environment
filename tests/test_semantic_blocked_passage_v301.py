"""Unit tests for V3.0.1 blocked_passage belief validation and store (no ROS)."""

from __future__ import annotations

import uuid
from datetime import datetime, timedelta, timezone

import pytest

from multi_robot_mission_stack.semantic.blocked_passage_local_stub_v301 import (
    build_blocked_passage_record_stub,
)
from multi_robot_mission_stack.semantic.blocked_passage_v301 import (
    BLOCKED_OUTCOME_VALUE,
    BLOCKED_REASON_CODE,
    BlockedPassageBeliefStore,
    SCHEMA_VERSION,
    TTL_SKEW_ALLOWANCE_SEC,
    make_blocked_by_peer_belief_outcome,
    validate_blocked_passage_record,
)


def _uuid4() -> str:
    return "a1b2c3d4-e5f6-4789-a012-3456789abcde"


def _uuid4_b() -> str:
    return "b2c3d4e5-f6a7-4890-b123-456789abcdef"


def _valid_record(
    *,
    belief_id: str | None = None,
    ts: str | None = None,
    ttl: float = 120.0,
    location: str = "base",
) -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "belief_id": belief_id or _uuid4(),
        "fact_type": "blocked_passage",
        "source_robot_id": "robot1",
        "timestamp_utc": ts or "2026-04-02T18:30:00.000Z",
        "confidence": 0.85,
        "location_ref": location,
        "provenance": {
            "sensor_class": "lidar_occlusion",
            "observation_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
        },
        "ttl_sec": ttl,
        "verification_status": "unverified",
    }


def test_validate_ok_canonical() -> None:
    ok, errs = validate_blocked_passage_record(_valid_record())
    assert ok and errs == []


def test_validate_rejects_bad_schema_version() -> None:
    r = _valid_record()
    r["schema_version"] = "v9"
    ok, errs = validate_blocked_passage_record(r)
    assert not ok
    assert any("schema_version" in e for e in errs)


def test_validate_rejects_non_uuid4_belief_id() -> None:
    r = _valid_record()
    r["belief_id"] = str(uuid.uuid3(uuid.NAMESPACE_DNS, "not-v4"))
    ok, errs = validate_blocked_passage_record(r)
    assert not ok
    assert any("belief_id" in e for e in errs)


def test_validate_rejects_confidence_out_of_range() -> None:
    r = _valid_record()
    r["confidence"] = 1.01
    ok, errs = validate_blocked_passage_record(r)
    assert not ok


def test_validate_rejects_bool_confidence() -> None:
    r = _valid_record()
    r["confidence"] = True  # type: ignore[assignment]
    ok, errs = validate_blocked_passage_record(r)
    assert not ok


def test_validate_rejects_missing_provenance_field() -> None:
    r = _valid_record()
    r["provenance"] = {"sensor_class": "x"}  # missing observation_id
    ok, errs = validate_blocked_passage_record(r)
    assert not ok


def test_validate_rejects_wrong_verification_status() -> None:
    r = _valid_record()
    r["verification_status"] = "confirmed"
    ok, errs = validate_blocked_passage_record(r)
    assert not ok


def test_ingest_stores_when_active() -> None:
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    rec = _valid_record(ts="2026-04-02T18:30:00.000Z", ttl=120.0)
    res = store.ingest(rec, now_utc=t0)
    assert res.stored and not res.rejected and not res.duplicate_ignored
    assert len(store) == 1


def test_ingest_rejects_expired_at_ingest() -> None:
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    rec = _valid_record(ts="2026-04-02T18:30:00.000Z", ttl=1.0)
    # just past deadline: start + 1 + 0.5 skew = 18:31:30.5
    late = datetime(2026, 4, 2, 18, 31, 31, tzinfo=timezone.utc)
    res = store.ingest(rec, now_utc=late)
    assert res.rejected and not res.stored
    assert "expired" in " ".join(res.errors).lower()


def test_ingest_duplicate_ignored() -> None:
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    bid = _uuid4()
    rec = _valid_record(belief_id=bid)
    assert store.ingest(rec, now_utc=t0).stored
    res2 = store.ingest(rec, now_utc=t0)
    assert res2.duplicate_ignored and not res2.stored
    assert len(store) == 1


def test_ingest_rejects_source_not_in_allowlist() -> None:
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot2"}))
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    res = store.ingest(_valid_record(), now_utc=t0)
    assert res.rejected and "allowlist" in res.errors[0].lower()


def test_has_active_blocked_passage_true() -> None:
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    store.ingest(_valid_record(location="corridor_A"), now_utc=t0)
    q = store.has_active_blocked_passage("corridor_A", now_utc=t0)
    assert q.has_active
    assert len(q.active_belief_ids) == 1


def test_has_active_false_after_ttl() -> None:
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    store.ingest(_valid_record(ts="2026-04-02T18:30:00.000Z", ttl=60.0), now_utc=t0)
    late = t0 + timedelta(seconds=61 + TTL_SKEW_ALLOWANCE_SEC + 0.1)
    q = store.has_active_blocked_passage("base", now_utc=late)
    assert not q.has_active
    assert q.active_belief_ids == ()


def test_multiple_active_same_location_multiple_beliefs() -> None:
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    r1 = _valid_record(belief_id=_uuid4(), location="base")
    r2 = _valid_record(belief_id=_uuid4_b(), location="base")
    # second needs different timestamp to be valid sequential ingest - same time ok
    assert store.ingest(r1, now_utc=t0).stored
    assert store.ingest(r2, now_utc=t0).stored
    q = store.has_active_blocked_passage("base", now_utc=t0)
    assert q.has_active and len(q.active_belief_ids) == 2


def test_make_blocked_outcome_shape() -> None:
    out = make_blocked_by_peer_belief_outcome(
        requested_location_name="base",
        active_belief_ids=("id1", "id2"),
    )
    assert out["outcome"] == BLOCKED_OUTCOME_VALUE
    assert out["reason_code"] == BLOCKED_REASON_CODE
    assert out["requested_location_name"] == "base"
    assert out["active_belief_ids"] == ["id1", "id2"]
    assert out["schema_version"] == SCHEMA_VERSION


def test_ttl_skew_extends_active_window() -> None:
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    start = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    rec = _valid_record(ts="2026-04-02T18:30:00.000Z", ttl=60.0)
    store.ingest(rec, now_utc=start)
    # 60 + 0.5 skew after start -> still active just inside skew tail
    edge = start + timedelta(seconds=60.25)
    assert store.has_active_blocked_passage("base", now_utc=edge).has_active
    past = start + timedelta(seconds=61.0)
    assert not store.has_active_blocked_passage("base", now_utc=past).has_active


def _stub_kwargs(
    *,
    t0: datetime | None = None,
    belief_id: str | None = None,
    ttl_sec: float = 120.0,
    location_ref: str = "base",
    confidence: float = 0.85,
) -> dict:
    ts = t0 or datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    return {
        "belief_id": belief_id or _uuid4(),
        "source_robot_id": "robot1",
        "location_ref": location_ref,
        "confidence": confidence,
        "timestamp_utc": ts,
        "ttl_sec": ttl_sec,
        "sensor_class": "local_stub",
        "observation_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
    }


def test_stub_build_passes_validate_blocked_passage_record() -> None:
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    rec = build_blocked_passage_record_stub(**_stub_kwargs(t0=t0))
    ok, errs = validate_blocked_passage_record(rec)
    assert ok and errs == []


def test_stub_ingest_succeeds_for_fresh_fact() -> None:
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    rec = build_blocked_passage_record_stub(**_stub_kwargs(t0=t0))
    res = store.ingest(rec, now_utc=t0)
    assert res.stored and not res.rejected


def test_stub_rejects_naive_timestamp_utc() -> None:
    naive = datetime(2026, 4, 2, 18, 30, 0)
    with pytest.raises(ValueError, match="timezone-aware"):
        build_blocked_passage_record_stub(**_stub_kwargs(t0=naive))


def test_stub_rejects_invalid_belief_id_via_validate() -> None:
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    bad_id = str(uuid.uuid3(uuid.NAMESPACE_DNS, "not-v4"))
    with pytest.raises(ValueError, match="belief_id"):
        build_blocked_passage_record_stub(**_stub_kwargs(t0=t0, belief_id=bad_id))


def test_stub_rejects_confidence_out_of_range() -> None:
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    with pytest.raises(ValueError, match="confidence"):
        build_blocked_passage_record_stub(**_stub_kwargs(t0=t0, confidence=1.5))


def test_stub_rejects_invalid_observation_id() -> None:
    t0 = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
    bad_obs = str(uuid.uuid3(uuid.NAMESPACE_DNS, "not-v4"))
    kwargs = _stub_kwargs(t0=t0)
    kwargs["observation_id"] = bad_obs
    with pytest.raises(ValueError, match="observation_id"):
        build_blocked_passage_record_stub(**kwargs)
