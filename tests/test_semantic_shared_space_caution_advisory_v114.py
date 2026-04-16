"""V11.4 local advisory query/reporting for shared_space_caution (no ROS, no execution)."""

from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from multi_robot_mission_stack.semantic.shared_space_caution_v111 import (
    ADVISORY_QUERY_REPORT_SCHEMA_VERSION,
    FACT_TYPE_SHARED_SPACE_CAUTION,
    SharedSpaceCautionBeliefStore,
    TTL_SKEW_ALLOWANCE_SEC,
)


def _load_v111_examples() -> dict[str, Any]:
    p = (
        Path(__file__).resolve().parent
        / "fixtures"
        / "semantic_v11_1_shared_space_caution_contract_examples.json"
    )
    return json.loads(p.read_text(encoding="utf-8"))


def _valid_record() -> dict[str, Any]:
    data = _load_v111_examples()
    return dict(data["cases"][0]["record"])


def test_v114_no_active_advisory_at_location() -> None:
    store = SharedSpaceCautionBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    now = datetime(2026, 4, 8, 12, 30, 30, tzinfo=timezone.utc)
    rep = store.query_advisory_shared_space_caution("base", now_utc=now)
    assert not rep.has_active
    assert rep.entries == ()
    assert rep.active_belief_ids == ()
    assert rep.location_ref == "base"
    assert rep.schema_version == ADVISORY_QUERY_REPORT_SCHEMA_VERSION
    assert rep.fact_type == FACT_TYPE_SHARED_SPACE_CAUTION
    d = rep.to_dict()
    assert d["has_active"] is False
    assert d["entries"] == []


def test_v114_one_active_advisory_entry() -> None:
    store = SharedSpaceCautionBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    now = datetime(2026, 4, 8, 12, 30, 30, tzinfo=timezone.utc)
    rec = _valid_record()
    assert store.ingest(rec, now_utc=now).stored
    rep = store.query_advisory_shared_space_caution("base", now_utc=now)
    assert rep.has_active
    assert len(rep.entries) == 1
    assert rep.active_belief_ids == (rec["belief_id"],)
    e = rep.entries[0]
    assert e.source_robot_id == "robot1"
    assert e.caution_class == "pedestrian_activity"
    assert e.confidence == 0.7
    d = rep.to_dict()
    assert d["schema_version"] == ADVISORY_QUERY_REPORT_SCHEMA_VERSION
    assert d["fact_type"] == FACT_TYPE_SHARED_SPACE_CAUTION
    assert len(d["entries"]) == 1
    assert d["entries"][0]["belief_id"] == rec["belief_id"]


def test_v114_expired_belief_not_reported_but_remains_in_store() -> None:
    store = SharedSpaceCautionBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    start = datetime(2026, 4, 8, 12, 30, 0, tzinfo=timezone.utc)
    rec = _valid_record()
    assert store.ingest(rec, now_utc=start).stored
    assert len(store) == 1

    outside = start + timedelta(seconds=120.0 + TTL_SKEW_ALLOWANCE_SEC + 0.1)
    rep = store.query_advisory_shared_space_caution("base", now_utc=outside)
    assert not rep.has_active
    assert rep.entries == ()
    assert len(store) == 1


def test_v114_duplicate_ingest_single_advisory_entry() -> None:
    store = SharedSpaceCautionBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    now = datetime(2026, 4, 8, 12, 30, 30, tzinfo=timezone.utc)
    rec = _valid_record()
    assert store.ingest(rec, now_utc=now).stored
    assert store.ingest(rec, now_utc=now).duplicate_ignored
    rep = store.query_advisory_shared_space_caution("base", now_utc=now)
    assert rep.has_active
    assert len(rep.entries) == 1
    assert len(rep.active_belief_ids) == 1


def test_v114_two_distinct_beliefs_sorted_by_belief_id() -> None:
    store = SharedSpaceCautionBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    now = datetime(2026, 4, 8, 12, 30, 30, tzinfo=timezone.utc)
    r1 = _valid_record()
    r2 = dict(r1)
    r2["belief_id"] = "223e4567-e89b-42d3-a456-426614174000"
    r2["provenance"] = {
        "sensor_class": "pedestrian_proxy",
        "observation_id": "323e4567-e89b-42d3-a456-426614174001",
    }
    assert store.ingest(r2, now_utc=now).stored
    assert store.ingest(r1, now_utc=now).stored
    rep = store.query_advisory_shared_space_caution("base", now_utc=now)
    assert rep.has_active
    assert len(rep.entries) == 2
    ids = [e.belief_id for e in rep.entries]
    assert ids == sorted(ids)
