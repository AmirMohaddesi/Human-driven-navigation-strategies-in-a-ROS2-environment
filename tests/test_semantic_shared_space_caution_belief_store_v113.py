"""V11.3 deterministic local belief-store tests for shared_space_caution."""

from __future__ import annotations

import json
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from multi_robot_mission_stack.semantic.shared_space_caution_v111 import (
    SharedSpaceCautionBeliefStore,
    is_shared_space_caution_active,
)


def _fixture_accept_record() -> dict[str, Any]:
    p = (
        Path(__file__).resolve().parent
        / "fixtures"
        / "semantic_v11_1_shared_space_caution_contract_examples.json"
    )
    data = json.loads(p.read_text(encoding="utf-8"))
    for case in data["cases"]:
        if case["case_id"] == "accept_minimal_valid":
            return dict(case["record"])
    raise AssertionError("missing accept_minimal_valid fixture")


def test_v113_ingest_accepts_valid_fixture_record() -> None:
    rec = _fixture_accept_record()
    store = SharedSpaceCautionBeliefStore()
    now = datetime(2026, 4, 8, 12, 29, 0, tzinfo=timezone.utc)
    res = store.ingest(rec, now_utc=now)
    assert res.stored and not res.rejected and not res.duplicate_ignored
    assert not res.errors
    assert len(store) == 1
    q = store.has_active_shared_space_caution("base", now_utc=now)
    assert q.has_active
    assert rec["belief_id"] in q.active_belief_ids


def test_v113_ingest_rejects_invalid_record() -> None:
    rec = _fixture_accept_record()
    rec["confidence"] = 2.0
    store = SharedSpaceCautionBeliefStore()
    now = datetime(2026, 4, 8, 12, 29, 0, tzinfo=timezone.utc)
    res = store.ingest(rec, now_utc=now)
    assert res.rejected and not res.stored
    assert res.errors
    assert len(store) == 0


def test_v113_ingest_duplicate_belief_id_ignored() -> None:
    rec = _fixture_accept_record()
    store = SharedSpaceCautionBeliefStore()
    now = datetime(2026, 4, 8, 12, 29, 0, tzinfo=timezone.utc)
    assert store.ingest(rec, now_utc=now).stored
    res2 = store.ingest(rec, now_utc=now)
    assert res2.duplicate_ignored and not res2.stored and not res2.rejected
    assert len(store) == 1


def test_v113_ttl_expired_at_ingest_rejected() -> None:
    rec = _fixture_accept_record()
    rec["ttl_sec"] = 10.0
    store = SharedSpaceCautionBeliefStore()
    now = datetime(2026, 4, 8, 12, 30, 15, tzinfo=timezone.utc)
    res = store.ingest(rec, now_utc=now)
    assert res.rejected and not res.stored
    assert any("expired" in e.lower() for e in res.errors)
    assert len(store) == 0


def test_v113_query_inactive_after_ttl_window() -> None:
    rec = _fixture_accept_record()
    rec["ttl_sec"] = 60.0
    store = SharedSpaceCautionBeliefStore()
    t0 = datetime(2026, 4, 8, 12, 30, 0, tzinfo=timezone.utc)
    assert store.ingest(rec, now_utc=t0).stored
    later = t0 + timedelta(seconds=61.0)
    q = store.has_active_shared_space_caution("base", now_utc=later)
    assert not q.has_active
    assert q.active_belief_ids == ()


def test_v113_allowlist_rejects_unknown_source() -> None:
    rec = _fixture_accept_record()
    rec["source_robot_id"] = "robot2"
    store = SharedSpaceCautionBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    now = datetime(2026, 4, 8, 12, 29, 0, tzinfo=timezone.utc)
    res = store.ingest(rec, now_utc=now)
    assert res.rejected and not res.stored
    assert any("allowlist" in e for e in res.errors)


def test_v113_is_active_matches_store_window() -> None:
    rec = _fixture_accept_record()
    rec["belief_id"] = str(uuid.uuid4())
    rec["ttl_sec"] = 1.0
    t0 = datetime(2026, 4, 8, 12, 30, 0, tzinfo=timezone.utc)
    assert is_shared_space_caution_active(rec, now_utc=t0)[0]
    past = t0 + timedelta(seconds=2.0)
    assert not is_shared_space_caution_active(rec, now_utc=past)[0]
