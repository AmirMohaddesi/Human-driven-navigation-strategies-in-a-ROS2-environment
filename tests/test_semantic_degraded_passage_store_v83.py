"""V8.3 deterministic local store/admission scaffold tests for degraded_passage."""

from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from multi_robot_mission_stack.semantic.degraded_passage_v81 import (
    DegradedPassageBeliefStore,
    TTL_SKEW_ALLOWANCE_SEC,
)


def _load_v81_examples() -> dict[str, Any]:
    p = (
        Path(__file__).resolve().parent
        / "fixtures"
        / "semantic_v8_1_degraded_passage_contract_examples.json"
    )
    return json.loads(p.read_text(encoding="utf-8"))


def _valid_record() -> dict[str, Any]:
    data = _load_v81_examples()
    return dict(data["cases"][0]["record"])


def test_v83_ingest_accepts_valid_active_record() -> None:
    store = DegradedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    now = datetime(2026, 4, 8, 12, 30, 30, tzinfo=timezone.utc)
    rec = _valid_record()
    res = store.ingest(rec, now_utc=now)
    assert res.stored and not res.rejected and not res.duplicate_ignored
    assert len(store) == 1


def test_v83_ingest_rejects_invalid_record() -> None:
    store = DegradedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    now = datetime(2026, 4, 8, 12, 30, 30, tzinfo=timezone.utc)
    rec = _valid_record()
    rec["recommended_speed_factor"] = 1.4
    res = store.ingest(rec, now_utc=now)
    assert res.rejected and not res.stored
    assert res.errors


def test_v83_ingest_duplicate_ignored() -> None:
    store = DegradedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    now = datetime(2026, 4, 8, 12, 30, 30, tzinfo=timezone.utc)
    rec = _valid_record()
    assert store.ingest(rec, now_utc=now).stored
    res2 = store.ingest(rec, now_utc=now)
    assert res2.duplicate_ignored and not res2.stored and not res2.rejected


def test_v83_ingest_rejects_expired_at_ingest() -> None:
    store = DegradedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    rec = _valid_record()
    # start is 12:30:00Z, ttl=120; expires after 12:32:00.5 with skew
    late = datetime(2026, 4, 8, 12, 32, 1, tzinfo=timezone.utc)
    res = store.ingest(rec, now_utc=late)
    assert res.rejected and not res.stored
    assert "expired" in " ".join(res.errors).lower()


def test_v83_ingest_enforces_source_allowlist() -> None:
    store = DegradedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot2"}))
    now = datetime(2026, 4, 8, 12, 30, 30, tzinfo=timezone.utc)
    res = store.ingest(_valid_record(), now_utc=now)
    assert res.rejected and not res.stored
    assert "allowlist" in " ".join(res.errors).lower()


def test_v83_active_query_false_after_ttl_window() -> None:
    store = DegradedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    start = datetime(2026, 4, 8, 12, 30, 0, tzinfo=timezone.utc)
    rec = _valid_record()
    assert store.ingest(rec, now_utc=start).stored

    inside = start + timedelta(seconds=120.0 + TTL_SKEW_ALLOWANCE_SEC - 0.1)
    assert store.has_active_degraded_passage("base", now_utc=inside).has_active

    outside = start + timedelta(seconds=120.0 + TTL_SKEW_ALLOWANCE_SEC + 0.1)
    q = store.has_active_degraded_passage("base", now_utc=outside)
    assert not q.has_active
    assert q.active_belief_ids == ()
