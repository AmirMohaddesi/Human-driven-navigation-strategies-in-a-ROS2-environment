"""V3.3.b evaluation harness — offline deterministic regression."""

from __future__ import annotations

import json
from datetime import datetime, timezone

import pytest

from multi_robot_mission_stack.semantic.llm_candidate_boundary_v33 import CANDIDATE_MALFORMED
from multi_robot_mission_stack.semantic.llm_candidate_eval_v33 import (
    OUTCOME_ACCEPTED_RECORD,
    OUTCOME_ADAPTER_FAILURE,
    OUTCOME_BOUNDARY_REJECT,
)
from multi_robot_mission_stack.semantic.llm_eval_harness_v33b import (
    DEFAULT_FIXTURES_V33B,
    LlmEvalFixtureV33B,
    run_llm_eval_harness_v33b,
)
from multi_robot_mission_stack.semantic.llm_real_adapter_v33 import (
    ADAPTER_PROVIDER_ERROR,
    LlmAdapterFailure,
    LlmAdapterSuccess,
)

TS = datetime(2025, 1, 15, 10, 0, 0, tzinfo=timezone.utc)


def _ctx_base():
    return {
        "schema_version": "v3.3.llm_context.1",
        "location_ref": "base",
        "source_robot_id": "r1",
        "nav_goal_status": "active",
        "stall_duration_sec": 1.0,
        "planner_status": "computing",
        "lidar_occlusion_proxy": True,
        "operator_hint": "",
    }


def test_harness_all_accepted():
    def adapter(ctx):
        loc = str(ctx["location_ref"]).strip()
        return LlmAdapterSuccess(
            raw_text=json.dumps(
                {
                    "schema_version": "v3.3.llm_candidate.1",
                    "assert_blocked": True,
                    "location_ref": loc,
                    "confidence": 0.6,
                    "ttl_sec": 60.0,
                    "sensor_class": "x",
                }
            )
        )

    summary = run_llm_eval_harness_v33b(
        DEFAULT_FIXTURES_V33B,
        adapter_call=adapter,
        trials_per_fixture=2,
        default_timestamp_utc=TS,
    )
    assert summary.total_trials == len(DEFAULT_FIXTURES_V33B) * 2
    assert summary.accepted_count == summary.total_trials
    assert summary.boundary_reject_count == 0
    assert summary.final_schema_reject_count == 0
    assert summary.adapter_failure_count == 0
    d = summary.to_dict()
    assert d["accepted_rate"] == 1.0
    assert len(d["per_fixture"]) == len(DEFAULT_FIXTURES_V33B)
    for row in d["per_fixture"]:
        assert row["trials"] == 2
        assert row["accepted_count"] == 2
        assert row["accepted_rate"] == 1.0


def test_harness_adapter_failure_counts():
    def adapter(_ctx):
        return LlmAdapterFailure(ADAPTER_PROVIDER_ERROR, "unavailable")

    summary = run_llm_eval_harness_v33b(
        (LlmEvalFixtureV33B(name="only", llm_context=_ctx_base()),),
        adapter_call=adapter,
        trials_per_fixture=3,
        default_timestamp_utc=TS,
    )
    assert summary.total_trials == 3
    assert summary.adapter_failure_count == 3
    assert summary.adapter_reason_counts.get(ADAPTER_PROVIDER_ERROR) == 3


def test_harness_boundary_reject_and_reason_aggregation():
    def adapter(_ctx):
        return LlmAdapterSuccess(raw_text="not valid json {")

    summary = run_llm_eval_harness_v33b(
        (
            LlmEvalFixtureV33B(name="a", llm_context=_ctx_base()),
            LlmEvalFixtureV33B(name="b", llm_context=_ctx_base()),
        ),
        adapter_call=adapter,
        trials_per_fixture=2,
        default_timestamp_utc=TS,
    )
    assert summary.total_trials == 4
    assert summary.boundary_reject_count == 4
    assert summary.boundary_reason_counts.get(CANDIDATE_MALFORMED) == 4
    assert summary.accepted_count == 0


def test_harness_mixed_outcomes_alternating():
    n = {"i": 0}

    def adapter(ctx):
        n["i"] += 1
        if n["i"] % 2 == 1:
            return LlmAdapterFailure(ADAPTER_PROVIDER_ERROR, "x")
        loc = str(ctx["location_ref"]).strip()
        return LlmAdapterSuccess(
            raw_text=json.dumps(
                {
                    "schema_version": "v3.3.llm_candidate.1",
                    "assert_blocked": True,
                    "location_ref": loc,
                    "confidence": 0.5,
                    "ttl_sec": 10.0,
                    "sensor_class": "a",
                }
            )
        )

    summary = run_llm_eval_harness_v33b(
        (LlmEvalFixtureV33B(name="one", llm_context=_ctx_base()),),
        adapter_call=adapter,
        trials_per_fixture=4,
        default_timestamp_utc=TS,
    )
    assert summary.total_trials == 4
    assert summary.adapter_failure_count == 2
    assert summary.accepted_count == 2


def test_harness_trials_per_fixture_lt_one_raises():
    with pytest.raises(ValueError, match="trials_per_fixture"):
        run_llm_eval_harness_v33b(
            DEFAULT_FIXTURES_V33B,
            adapter_call=lambda c: LlmAdapterSuccess(raw_text="{}"),
            trials_per_fixture=0,
        )


def test_harness_empty_fixtures_raises():
    with pytest.raises(ValueError, match="non-empty"):
        run_llm_eval_harness_v33b(
            [],
            adapter_call=lambda c: LlmAdapterSuccess(raw_text="{}"),
            trials_per_fixture=1,
        )


def test_fixture_overrides_timestamp():
    custom_ts = datetime(2020, 1, 1, 0, 0, 0, tzinfo=timezone.utc)

    def adapter(ctx):
        loc = str(ctx["location_ref"]).strip()
        return LlmAdapterSuccess(
            raw_text=json.dumps(
                {
                    "schema_version": "v3.3.llm_candidate.1",
                    "assert_blocked": True,
                    "location_ref": loc,
                    "confidence": 0.5,
                    "ttl_sec": 10.0,
                    "sensor_class": "a",
                }
            )
        )

    fx = LlmEvalFixtureV33B(name="ts", llm_context=_ctx_base(), timestamp_utc=custom_ts)
    summary = run_llm_eval_harness_v33b(
        (fx,),
        adapter_call=adapter,
        trials_per_fixture=1,
        default_timestamp_utc=TS,
    )
    assert summary.accepted_count == 1
    # Record timestamp comes from evaluate path — spot-check one trial would need capturing;
    # here we only ensure harness runs with override without error.
    assert summary.total_trials == 1
