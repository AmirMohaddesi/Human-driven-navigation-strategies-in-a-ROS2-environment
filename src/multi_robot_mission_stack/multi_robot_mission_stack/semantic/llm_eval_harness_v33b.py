"""
V3.3.b — bounded local evaluation harness: repeated adapter-backed trials over fixed fixtures.

Measurement only: counts outcomes through the unchanged V3.3.a pipeline. No contract changes.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from .llm_candidate_eval_v33 import (
    OUTCOME_ACCEPTED_RECORD,
    OUTCOME_ADAPTER_FAILURE,
    OUTCOME_BOUNDARY_REJECT,
    OUTCOME_FINAL_SCHEMA_REJECT,
    LlmLocalEvalResult,
    evaluate_llm_candidate_via_adapter_v33,
)
from .llm_real_adapter_v33 import LlmAdapterResult

# --- Fixtures (fixed, explicit) ---


@dataclass(frozen=True)
class LlmEvalFixtureV33B:
    """One evaluation case: allowlisted context plus metadata."""

    name: str
    llm_context: Mapping[str, Any]
    notes: str = ""
    timestamp_utc: Optional[datetime] = None


DEFAULT_TIMESTAMP_V33B = datetime(2024, 6, 1, 12, 0, 0, tzinfo=timezone.utc)

DEFAULT_FIXTURES_V33B: Tuple[LlmEvalFixtureV33B, ...] = (
    LlmEvalFixtureV33B(
        name="base_active_stall",
        notes="Active nav goal, computing planner, occlusion proxy on.",
        llm_context={
            "schema_version": "v3.3.llm_context.1",
            "location_ref": "base",
            "source_robot_id": "r1",
            "nav_goal_status": "active",
            "stall_duration_sec": 12.0,
            "planner_status": "computing",
            "lidar_occlusion_proxy": True,
            "operator_hint": "",
        },
    ),
    LlmEvalFixtureV33B(
        name="base_with_operator_hint",
        notes="Same location; bounded operator hint present.",
        llm_context={
            "schema_version": "v3.3.llm_context.1",
            "location_ref": "base",
            "source_robot_id": "r1",
            "nav_goal_status": "active",
            "stall_duration_sec": 30.0,
            "planner_status": "failed",
            "lidar_occlusion_proxy": False,
            "operator_hint": "corridor appears blocked",
        },
    ),
    LlmEvalFixtureV33B(
        name="warehouse_idle_low_stall",
        notes="Different location_ref; idle planner.",
        llm_context={
            "schema_version": "v3.3.llm_context.1",
            "location_ref": "warehouse_door",
            "source_robot_id": "r2",
            "nav_goal_status": "pending",
            "stall_duration_sec": 2.5,
            "planner_status": "idle",
            "lidar_occlusion_proxy": False,
            "operator_hint": "",
        },
    ),
)


# --- Summary types ---


@dataclass(frozen=True)
class LlmEvalPerFixtureStatsV33B:
    fixture_name: str
    trials: int
    accepted_count: int
    boundary_reject_count: int
    final_schema_reject_count: int
    adapter_failure_count: int
    boundary_reason_counts: Dict[str, int]
    adapter_reason_counts: Dict[str, int]
    assembly_reason_counts: Dict[str, int]

    def to_dict(self) -> Dict[str, Any]:
        tr = self.trials
        return {
            "fixture_name": self.fixture_name,
            "trials": tr,
            "accepted_count": self.accepted_count,
            "boundary_reject_count": self.boundary_reject_count,
            "final_schema_reject_count": self.final_schema_reject_count,
            "adapter_failure_count": self.adapter_failure_count,
            "accepted_rate": (self.accepted_count / tr) if tr else 0.0,
            "boundary_reason_counts": dict(self.boundary_reason_counts),
            "adapter_reason_counts": dict(self.adapter_reason_counts),
            "assembly_reason_counts": dict(self.assembly_reason_counts),
        }


@dataclass(frozen=True)
class LlmEvalHarnessSummaryV33B:
    """Aggregated counts over all fixtures × trials."""

    total_trials: int
    accepted_count: int
    boundary_reject_count: int
    final_schema_reject_count: int
    adapter_failure_count: int
    boundary_reason_counts: Dict[str, int]
    adapter_reason_counts: Dict[str, int]
    assembly_reason_counts: Dict[str, int]
    per_fixture: Tuple[LlmEvalPerFixtureStatsV33B, ...]
    harness_notes: str = (
        "Descriptive measurement only; not operational readiness or model quality proof."
    )

    def to_dict(self) -> Dict[str, Any]:
        tt = self.total_trials
        return {
            "total_trials": tt,
            "accepted_count": self.accepted_count,
            "boundary_reject_count": self.boundary_reject_count,
            "final_schema_reject_count": self.final_schema_reject_count,
            "adapter_failure_count": self.adapter_failure_count,
            "accepted_rate": (self.accepted_count / tt) if tt else 0.0,
            "boundary_reject_rate": (self.boundary_reject_count / tt) if tt else 0.0,
            "final_schema_reject_rate": (self.final_schema_reject_count / tt) if tt else 0.0,
            "adapter_failure_rate": (self.adapter_failure_count / tt) if tt else 0.0,
            "boundary_reason_counts": dict(self.boundary_reason_counts),
            "adapter_reason_counts": dict(self.adapter_reason_counts),
            "assembly_reason_counts": dict(self.assembly_reason_counts),
            "per_fixture": [p.to_dict() for p in self.per_fixture],
            "harness_notes": self.harness_notes,
        }


def _accumulate_result(
    r: LlmLocalEvalResult,
    *,
    outcome_counter: Counter[str],
    boundary_reason_counter: Counter[str],
    adapter_reason_counter: Counter[str],
    assembly_reason_counter: Counter[str],
) -> None:
    outcome_counter[r.outcome] += 1
    if r.outcome == OUTCOME_BOUNDARY_REJECT and r.boundary_reason:
        boundary_reason_counter[r.boundary_reason] += 1
    if r.outcome == OUTCOME_ADAPTER_FAILURE and r.adapter_reason:
        adapter_reason_counter[r.adapter_reason] += 1
    if r.outcome == OUTCOME_FINAL_SCHEMA_REJECT and r.assembly_reason:
        assembly_reason_counter[r.assembly_reason] += 1


def run_llm_eval_harness_v33b(
    fixtures: Sequence[LlmEvalFixtureV33B],
    *,
    adapter_call: Callable[[Mapping[str, Any]], LlmAdapterResult],
    trials_per_fixture: int = 1,
    default_timestamp_utc: Optional[datetime] = None,
) -> LlmEvalHarnessSummaryV33B:
    """
    For each fixture, run ``trials_per_fixture`` calls to ``evaluate_llm_candidate_via_adapter_v33``
    with the same frozen pipeline. Aggregates outcome and reason counts only.
    """
    if trials_per_fixture < 1:
        raise ValueError("trials_per_fixture must be >= 1")
    if not fixtures:
        raise ValueError("fixtures must be non-empty")

    ts_default = default_timestamp_utc or DEFAULT_TIMESTAMP_V33B

    total_outcomes: Counter[str] = Counter()
    total_boundary: Counter[str] = Counter()
    total_adapter: Counter[str] = Counter()
    total_assembly: Counter[str] = Counter()

    per_fixture_rows: List[LlmEvalPerFixtureStatsV33B] = []

    for fx in fixtures:
        ts = fx.timestamp_utc or ts_default
        oc: Counter[str] = Counter()
        br: Counter[str] = Counter()
        ar: Counter[str] = Counter()
        asr: Counter[str] = Counter()

        for _ in range(trials_per_fixture):
            result = evaluate_llm_candidate_via_adapter_v33(
                fx.llm_context,
                timestamp_utc=ts,
                adapter_call=adapter_call,
            )
            _accumulate_result(
                result,
                outcome_counter=oc,
                boundary_reason_counter=br,
                adapter_reason_counter=ar,
                assembly_reason_counter=asr,
            )
            _accumulate_result(
                result,
                outcome_counter=total_outcomes,
                boundary_reason_counter=total_boundary,
                adapter_reason_counter=total_adapter,
                assembly_reason_counter=total_assembly,
            )

        per_fixture_rows.append(
            LlmEvalPerFixtureStatsV33B(
                fixture_name=fx.name,
                trials=trials_per_fixture,
                accepted_count=oc[OUTCOME_ACCEPTED_RECORD],
                boundary_reject_count=oc[OUTCOME_BOUNDARY_REJECT],
                final_schema_reject_count=oc[OUTCOME_FINAL_SCHEMA_REJECT],
                adapter_failure_count=oc[OUTCOME_ADAPTER_FAILURE],
                boundary_reason_counts=dict(br),
                adapter_reason_counts=dict(ar),
                assembly_reason_counts=dict(asr),
            )
        )

    total_trials = len(fixtures) * trials_per_fixture

    return LlmEvalHarnessSummaryV33B(
        total_trials=total_trials,
        accepted_count=total_outcomes[OUTCOME_ACCEPTED_RECORD],
        boundary_reject_count=total_outcomes[OUTCOME_BOUNDARY_REJECT],
        final_schema_reject_count=total_outcomes[OUTCOME_FINAL_SCHEMA_REJECT],
        adapter_failure_count=total_outcomes[OUTCOME_ADAPTER_FAILURE],
        boundary_reason_counts=dict(total_boundary),
        adapter_reason_counts=dict(total_adapter),
        assembly_reason_counts=dict(total_assembly),
        per_fixture=tuple(per_fixture_rows),
    )
