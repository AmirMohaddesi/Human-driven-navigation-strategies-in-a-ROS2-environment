"""V3.3.a adapter seam + pipeline (deterministic tests; optional live OpenAI)."""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone

import pytest

from multi_robot_mission_stack.semantic.blocked_passage_v301 import validate_blocked_passage_record
from multi_robot_mission_stack.semantic.llm_candidate_boundary_v33 import CANDIDATE_MALFORMED
from multi_robot_mission_stack.semantic.llm_candidate_eval_v33 import (
    OUTCOME_ACCEPTED_RECORD,
    OUTCOME_ADAPTER_FAILURE,
    OUTCOME_BOUNDARY_REJECT,
    evaluate_llm_candidate_via_adapter_v33,
)
from multi_robot_mission_stack.semantic.llm_real_adapter_v33 import (
    ADAPTER_EMPTY_OUTPUT,
    ADAPTER_PROVIDER_ERROR,
    ADAPTER_TIMEOUT,
    LlmAdapterFailure,
    LlmAdapterSuccess,
    OpenAiChatCompletionsAdapterV33,
)

FIXED_TS = datetime(2024, 6, 1, 12, 0, 0, tzinfo=timezone.utc)
VALID_BELIEF = "12345678-1234-4678-8234-567812345678"
VALID_OBS = "87654321-4321-4678-8234-567812345678"


@pytest.fixture
def ctx():
    return {
        "schema_version": "v3.3.llm_context.1",
        "location_ref": "base",
        "source_robot_id": "r1",
        "nav_goal_status": "active",
        "stall_duration_sec": 12.0,
        "planner_status": "computing",
        "lidar_occlusion_proxy": True,
        "operator_hint": "",
    }


def _good_candidate():
    return {
        "schema_version": "v3.3.llm_candidate.1",
        "assert_blocked": True,
        "location_ref": "base",
        "confidence": 0.7,
        "ttl_sec": 3600.0,
        "sensor_class": "lidar_occlusion",
    }


def test_pipeline_fake_adapter_accepted_record(ctx):
    def adapter(_):
        return LlmAdapterSuccess(raw_text=json.dumps(_good_candidate()))

    r = evaluate_llm_candidate_via_adapter_v33(
        ctx,
        timestamp_utc=FIXED_TS,
        adapter_call=adapter,
        belief_id=VALID_BELIEF,
        observation_id=VALID_OBS,
    )
    assert r.outcome == OUTCOME_ACCEPTED_RECORD
    assert r.adapter_reason is None
    ok, _ = validate_blocked_passage_record(r.record or {})
    assert ok


def test_pipeline_fake_adapter_failure(ctx):
    def adapter(_):
        return LlmAdapterFailure(ADAPTER_TIMEOUT, "connection reset")

    r = evaluate_llm_candidate_via_adapter_v33(
        ctx,
        timestamp_utc=FIXED_TS,
        adapter_call=adapter,
    )
    assert r.outcome == OUTCOME_ADAPTER_FAILURE
    assert r.adapter_reason == ADAPTER_TIMEOUT
    assert "reset" in r.adapter_detail
    assert r.record is None


def test_pipeline_empty_model_text_is_adapter_failure(ctx):
    def adapter(_):
        return LlmAdapterSuccess(raw_text="   \n  ")

    r = evaluate_llm_candidate_via_adapter_v33(
        ctx,
        timestamp_utc=FIXED_TS,
        adapter_call=adapter,
    )
    assert r.outcome == OUTCOME_ADAPTER_FAILURE
    assert r.adapter_reason == ADAPTER_EMPTY_OUTPUT


def test_pipeline_noisy_model_text_is_boundary_reject(ctx):
    def adapter(_):
        return LlmAdapterSuccess(raw_text="Here is JSON: {...}")

    r = evaluate_llm_candidate_via_adapter_v33(
        ctx,
        timestamp_utc=FIXED_TS,
        adapter_call=adapter,
    )
    assert r.outcome == OUTCOME_BOUNDARY_REJECT
    assert r.boundary_reason == CANDIDATE_MALFORMED


def test_openai_adapter_reports_missing_key(ctx, monkeypatch):
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    adapter = OpenAiChatCompletionsAdapterV33(api_key="")
    out = adapter(ctx)
    assert isinstance(out, LlmAdapterFailure)
    assert out.reason == ADAPTER_PROVIDER_ERROR


@pytest.mark.skipif(
    os.environ.get("HDNS_V33A_LIVE_ADAPTER", "").strip() != "1",
    reason="set HDNS_V33A_LIVE_ADAPTER=1 and OPENAI_API_KEY to run live provider test",
)
def test_live_openai_adapter_optional_end_to_end(ctx):
    if not os.environ.get("OPENAI_API_KEY", "").strip():
        pytest.skip("OPENAI_API_KEY not set")
    adapter = OpenAiChatCompletionsAdapterV33(timeout_sec=90.0)
    r = evaluate_llm_candidate_via_adapter_v33(
        ctx,
        timestamp_utc=FIXED_TS,
        adapter_call=adapter,
        belief_id=VALID_BELIEF,
        observation_id=VALID_OBS,
    )
    # Real models may legitimately hit boundary or semantic reject; adapter must not crash.
    assert r.outcome in (
        OUTCOME_ACCEPTED_RECORD,
        OUTCOME_BOUNDARY_REJECT,
        OUTCOME_ADAPTER_FAILURE,
    )
    if r.outcome == OUTCOME_ACCEPTED_RECORD:
        ok, errs = validate_blocked_passage_record(r.record or {})
        assert ok, errs
