"""
V3.4 — bounded production-to-ingest proof: frozen V3.3 gate stack → existing ``BlockedPassageBeliefStore.ingest``.

Does not modify schema, store semantics, boundary, assembler, or adapter modules. No ROS/transport.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Any, Callable, Dict, Mapping, Optional, Tuple

from .blocked_passage_v301 import BlockedPassageBeliefStore, IngestResult
from .llm_candidate_eval_v33 import (
    OUTCOME_ACCEPTED_RECORD,
    LlmLocalEvalResult,
    evaluate_llm_candidate_via_adapter_v33,
)
from .llm_real_adapter_v33 import LlmAdapterResult

# Terminal outcomes after optional ingest (V3.4)
OUTCOME_INGEST_STORED = "ingest_stored"
OUTCOME_INGEST_DUPLICATE_IGNORED = "ingest_duplicate_ignored"
OUTCOME_INGEST_REJECTED = "ingest_rejected"


@dataclass(frozen=True)
class SemanticProductionIngestResultV34:
    """
    Result after adapter-backed gates and, when ``accepted_record``, ``store.ingest``.

    For gate failures, ``outcome`` matches ``llm_candidate_eval_v33`` (e.g. ``boundary_reject``).
    """

    outcome: str
    record: Optional[Dict[str, Any]] = None
    boundary_reason: Optional[str] = None
    boundary_detail: Optional[str] = None
    assembly_reason: Optional[str] = None
    assembly_detail: Optional[str] = None
    adapter_reason: Optional[str] = None
    adapter_detail: Optional[str] = None
    ingest_errors: Tuple[str, ...] = ()


def _from_gate_only(ev: LlmLocalEvalResult) -> SemanticProductionIngestResultV34:
    return SemanticProductionIngestResultV34(
        outcome=ev.outcome,
        record=ev.record,
        boundary_reason=ev.boundary_reason,
        boundary_detail=ev.boundary_detail,
        assembly_reason=ev.assembly_reason,
        assembly_detail=ev.assembly_detail,
        adapter_reason=ev.adapter_reason,
        adapter_detail=ev.adapter_detail,
        ingest_errors=(),
    )


def produce_and_ingest_blocked_passage_v34(
    llm_context: Mapping[str, Any],
    *,
    store: BlockedPassageBeliefStore,
    adapter_call: Callable[[Mapping[str, Any]], LlmAdapterResult],
    timestamp_utc: datetime,
    now_utc_ingest: datetime,
    belief_id: Optional[str] = None,
    observation_id: Optional[str] = None,
) -> SemanticProductionIngestResultV34:
    """
    Run the unchanged V3.3.a pipeline; on ``accepted_record`` only, call ``store.ingest`` unchanged.

    ``timestamp_utc`` is the explicit assembly clock for the final record. ``now_utc_ingest`` is
    the explicit ingest / TTL-evaluation clock (V3.0.1 semantics).
    """
    ev = evaluate_llm_candidate_via_adapter_v33(
        llm_context,
        timestamp_utc=timestamp_utc,
        adapter_call=adapter_call,
        belief_id=belief_id,
        observation_id=observation_id,
    )
    if ev.outcome != OUTCOME_ACCEPTED_RECORD:
        return _from_gate_only(ev)

    assert ev.record is not None
    ing: IngestResult = store.ingest(ev.record, now_utc=now_utc_ingest)

    if ing.stored:
        return SemanticProductionIngestResultV34(
            outcome=OUTCOME_INGEST_STORED,
            record=ev.record,
            ingest_errors=(),
        )
    if ing.duplicate_ignored:
        return SemanticProductionIngestResultV34(
            outcome=OUTCOME_INGEST_DUPLICATE_IGNORED,
            record=ev.record,
            ingest_errors=(),
        )
    return SemanticProductionIngestResultV34(
        outcome=OUTCOME_INGEST_REJECTED,
        record=ev.record,
        ingest_errors=tuple(ing.errors),
    )
