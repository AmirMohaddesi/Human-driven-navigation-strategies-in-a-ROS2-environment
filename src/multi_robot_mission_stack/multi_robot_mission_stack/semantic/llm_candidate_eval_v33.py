"""
V3.3 local evaluation path: allowlisted context + raw model text → boundary → assembly → outcome.

Pure Python seam for tests and model adapters. ``evaluate_llm_candidate_via_adapter_v33`` is
V3.3.a (adapter → unchanged gates). No ROS, no store/transport.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Any, Callable, Dict, Mapping, Optional

from .llm_blocked_passage_assembler_v33 import assemble_blocked_passage_from_v33_accept
from .llm_candidate_boundary_v33 import (
    LlmCandidateAccept,
    LlmCandidateReject,
    evaluate_llm_candidate_boundary,
)
from .llm_real_adapter_v33 import (
    ADAPTER_EMPTY_OUTPUT,
    LlmAdapterFailure,
    LlmAdapterResult,
    LlmAdapterSuccess,
)

# Local outcome labels (eval / logging; not wire protocol)
OUTCOME_ACCEPTED_RECORD = "accepted_record"
OUTCOME_BOUNDARY_REJECT = "boundary_reject"
OUTCOME_FINAL_SCHEMA_REJECT = "final_schema_reject"
OUTCOME_ADAPTER_FAILURE = "adapter_failure"


@dataclass(frozen=True)
class LlmLocalEvalResult:
    """Structured result after boundary + optional assembly (+ optional adapter stage)."""

    outcome: str
    record: Optional[Dict[str, Any]] = None
    boundary_reason: Optional[str] = None
    boundary_detail: Optional[str] = None
    assembly_reason: Optional[str] = None
    assembly_detail: Optional[str] = None
    adapter_reason: Optional[str] = None
    adapter_detail: Optional[str] = None


def evaluate_llm_candidate_local_v33(
    llm_context: Mapping[str, Any],
    simulated_model_output: str,
    *,
    timestamp_utc: datetime,
    belief_id: Optional[str] = None,
    observation_id: Optional[str] = None,
) -> LlmLocalEvalResult:
    """
    Run boundary evaluation, then deterministic assembly when the boundary accepts.

    ``simulated_model_output`` is the raw string a model adapter would pass through unchanged.
    ``belief_id`` / ``observation_id`` follow the assembler: omitted → generated UUID v4; set for
    deterministic or negative tests.

    ``candidate_semantically_insufficient`` (e.g. ``assert_blocked: false``) is reported as
    ``outcome=boundary_reject`` with that token in ``boundary_reason``.
    """
    boundary = evaluate_llm_candidate_boundary(llm_context, simulated_model_output)
    if isinstance(boundary, LlmCandidateReject):
        return LlmLocalEvalResult(
            outcome=OUTCOME_BOUNDARY_REJECT,
            boundary_reason=boundary.reason,
            boundary_detail=boundary.detail,
        )

    assert isinstance(boundary, LlmCandidateAccept)
    assembled = assemble_blocked_passage_from_v33_accept(
        boundary,
        timestamp_utc=timestamp_utc,
        belief_id=belief_id,
        observation_id=observation_id,
    )
    if not assembled.accepted:
        return LlmLocalEvalResult(
            outcome=OUTCOME_FINAL_SCHEMA_REJECT,
            assembly_reason=assembled.reason,
            assembly_detail=assembled.detail,
        )

    return LlmLocalEvalResult(
        outcome=OUTCOME_ACCEPTED_RECORD,
        record=assembled.record,
    )


def evaluate_llm_candidate_via_adapter_v33(
    llm_context: Mapping[str, Any],
    *,
    timestamp_utc: datetime,
    adapter_call: Callable[[Mapping[str, Any]], LlmAdapterResult],
    belief_id: Optional[str] = None,
    observation_id: Optional[str] = None,
) -> LlmLocalEvalResult:
    """
    V3.3.a: invoke ``adapter_call(llm_context)``, then run the unchanged local gate stack on
    the returned raw text. Adapter failures and empty content become ``adapter_failure`` without
    calling the boundary on empty strings (empty → ``ADAPTER_EMPTY_OUTPUT``).
    """
    adapter_out = adapter_call(llm_context)
    if isinstance(adapter_out, LlmAdapterFailure):
        return LlmLocalEvalResult(
            outcome=OUTCOME_ADAPTER_FAILURE,
            adapter_reason=adapter_out.reason,
            adapter_detail=adapter_out.detail,
        )

    assert isinstance(adapter_out, LlmAdapterSuccess)
    raw = adapter_out.raw_text
    if raw is None or not str(raw).strip():
        return LlmLocalEvalResult(
            outcome=OUTCOME_ADAPTER_FAILURE,
            adapter_reason=ADAPTER_EMPTY_OUTPUT,
            adapter_detail="model returned empty or whitespace-only content",
        )

    return evaluate_llm_candidate_local_v33(
        llm_context,
        raw,
        timestamp_utc=timestamp_utc,
        belief_id=belief_id,
        observation_id=observation_id,
    )
