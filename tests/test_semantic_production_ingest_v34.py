"""V3.4 production-to-ingest proof: gates → existing BlockedPassageBeliefStore.ingest (no ROS)."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from unittest.mock import MagicMock

from multi_robot_mission_stack.semantic.blocked_passage_v301 import BlockedPassageBeliefStore
from multi_robot_mission_stack.semantic.llm_candidate_eval_v33 import (
    OUTCOME_BOUNDARY_REJECT,
    OUTCOME_FINAL_SCHEMA_REJECT,
)
from multi_robot_mission_stack.semantic.llm_real_adapter_v33 import (
    ADAPTER_PROVIDER_ERROR,
    LlmAdapterFailure,
    LlmAdapterSuccess,
)
from multi_robot_mission_stack.semantic.semantic_production_ingest_v34 import (
    OUTCOME_INGEST_DUPLICATE_IGNORED,
    OUTCOME_INGEST_REJECTED,
    OUTCOME_INGEST_STORED,
    SemanticProductionIngestResultV34,
    produce_and_ingest_blocked_passage_v34,
)

TS_ASSEMBLY = datetime(2026, 4, 2, 18, 30, 0, tzinfo=timezone.utc)
TS_INGEST = datetime(2026, 4, 2, 18, 30, 30, tzinfo=timezone.utc)
BELIEF = "12345678-1234-4678-8234-567812345678"
OBS = "87654321-4321-4678-8234-567812345678"


def _ctx():
    return {
        "schema_version": "v3.3.llm_context.1",
        "location_ref": "base",
        "source_robot_id": "robot1",
        "nav_goal_status": "active",
        "stall_duration_sec": 5.0,
        "planner_status": "computing",
        "lidar_occlusion_proxy": True,
        "operator_hint": "",
    }


def _adapter_ok():
    def call(ctx):
        loc = str(ctx["location_ref"]).strip()
        return LlmAdapterSuccess(
            raw_text=json.dumps(
                {
                    "schema_version": "v3.3.llm_candidate.1",
                    "assert_blocked": True,
                    "location_ref": loc,
                    "confidence": 0.7,
                    "ttl_sec": 3600.0,
                    "sensor_class": "lidar_occlusion",
                }
            )
        )

    return call


def test_gate_failure_does_not_call_ingest():
    store = MagicMock(spec=BlockedPassageBeliefStore)
    store.ingest = MagicMock()

    def bad_adapter(_ctx):
        return LlmAdapterFailure(ADAPTER_PROVIDER_ERROR, "down")

    r = produce_and_ingest_blocked_passage_v34(
        _ctx(),
        store=store,  # type: ignore[arg-type]
        adapter_call=bad_adapter,
        timestamp_utc=TS_ASSEMBLY,
        now_utc_ingest=TS_INGEST,
    )
    assert isinstance(r, SemanticProductionIngestResultV34)
    store.ingest.assert_not_called()


def test_final_schema_reject_does_not_call_ingest():
    store = MagicMock(spec=BlockedPassageBeliefStore)
    store.ingest = MagicMock()

    r = produce_and_ingest_blocked_passage_v34(
        _ctx(),
        store=store,  # type: ignore[arg-type]
        adapter_call=_adapter_ok(),
        timestamp_utc=TS_ASSEMBLY,
        now_utc_ingest=TS_INGEST,
        belief_id="not-a-uuid-v4",
        observation_id=OBS,
    )
    assert r.outcome == OUTCOME_FINAL_SCHEMA_REJECT
    store.ingest.assert_not_called()


def test_boundary_reject_does_not_call_ingest():
    store = MagicMock(spec=BlockedPassageBeliefStore)
    store.ingest = MagicMock()

    def chatter(_ctx):
        return LlmAdapterSuccess(raw_text="not json {")

    r = produce_and_ingest_blocked_passage_v34(
        _ctx(),
        store=store,  # type: ignore[arg-type]
        adapter_call=chatter,
        timestamp_utc=TS_ASSEMBLY,
        now_utc_ingest=TS_INGEST,
    )
    assert r.outcome == OUTCOME_BOUNDARY_REJECT
    store.ingest.assert_not_called()


def test_accepted_record_ingest_stored():
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    r = produce_and_ingest_blocked_passage_v34(
        _ctx(),
        store=store,
        adapter_call=_adapter_ok(),
        timestamp_utc=TS_ASSEMBLY,
        now_utc_ingest=TS_INGEST,
        belief_id=BELIEF,
        observation_id=OBS,
    )
    assert r.outcome == OUTCOME_INGEST_STORED
    assert r.record is not None
    assert r.record["belief_id"] == BELIEF
    assert len(store) == 1
    got = store.get_record(BELIEF)
    assert got is not None
    assert got["location_ref"] == "base"


def test_duplicate_second_call_duplicate_ignored():
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    adapter = _adapter_ok()
    kw = dict(
        llm_context=_ctx(),
        store=store,
        adapter_call=adapter,
        timestamp_utc=TS_ASSEMBLY,
        now_utc_ingest=TS_INGEST,
        belief_id=BELIEF,
        observation_id=OBS,
    )
    r1 = produce_and_ingest_blocked_passage_v34(**kw)
    r2 = produce_and_ingest_blocked_passage_v34(**kw)
    assert r1.outcome == OUTCOME_INGEST_STORED
    assert r2.outcome == OUTCOME_INGEST_DUPLICATE_IGNORED
    assert len(store) == 1


def test_ingest_rejected_allowlist():
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"other_robot"}))
    r = produce_and_ingest_blocked_passage_v34(
        _ctx(),
        store=store,
        adapter_call=_adapter_ok(),
        timestamp_utc=TS_ASSEMBLY,
        now_utc_ingest=TS_INGEST,
        belief_id=BELIEF,
        observation_id=OBS,
    )
    assert r.outcome == OUTCOME_INGEST_REJECTED
    assert r.ingest_errors
    assert "allowlist" in r.ingest_errors[0].lower()
    assert len(store) == 0


def test_ingest_rejected_expired_at_ingest_time():
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    # Short TTL: assembly at 18:30, ingest clock past deadline (ttl 2s + skew 0.5)
    late = datetime(2026, 4, 2, 18, 30, 5, tzinfo=timezone.utc)

    def adapter_short_ttl(ctx):
        loc = str(ctx["location_ref"]).strip()
        return LlmAdapterSuccess(
            raw_text=json.dumps(
                {
                    "schema_version": "v3.3.llm_candidate.1",
                    "assert_blocked": True,
                    "location_ref": loc,
                    "confidence": 0.7,
                    "ttl_sec": 2.0,
                    "sensor_class": "lidar_occlusion",
                }
            )
        )

    r = produce_and_ingest_blocked_passage_v34(
        _ctx(),
        store=store,
        adapter_call=adapter_short_ttl,
        timestamp_utc=TS_ASSEMBLY,
        now_utc_ingest=late,
        belief_id=BELIEF,
        observation_id=OBS,
    )
    assert r.outcome == OUTCOME_INGEST_REJECTED
    assert any("expired" in e.lower() for e in r.ingest_errors)
    assert len(store) == 0


def test_has_active_blocked_passage_after_ingest():
    store = BlockedPassageBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    produce_and_ingest_blocked_passage_v34(
        _ctx(),
        store=store,
        adapter_call=_adapter_ok(),
        timestamp_utc=TS_ASSEMBLY,
        now_utc_ingest=TS_INGEST,
        belief_id=BELIEF,
        observation_id=OBS,
    )
    q = store.has_active_blocked_passage("base", now_utc=TS_INGEST)
    assert q.has_active
    assert BELIEF in q.active_belief_ids
