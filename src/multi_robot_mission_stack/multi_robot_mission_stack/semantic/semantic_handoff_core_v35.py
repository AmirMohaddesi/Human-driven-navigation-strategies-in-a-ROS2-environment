"""
V3.5 core — parse bounded ROS handoff JSON and run the frozen V3.4 production→ingest path.

No rclpy. Request wrapper keys are allowlisted; clocks are explicit ISO strings (no hidden wall clock
in this layer). Does not modify V3.3 boundary, assembler, adapter modules, or store ingest logic.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from typing import Any, Callable, Dict, Mapping, Optional, Tuple

from .llm_real_adapter_v33 import LlmAdapterResult, LlmAdapterSuccess, OpenAiChatCompletionsAdapterV33
from .semantic_production_ingest_v34 import (
    SemanticProductionIngestResultV34,
    produce_and_ingest_blocked_passage_v34,
)
from .blocked_passage_v301 import BlockedPassageBeliefStore

# V3.5 request envelope (strict key set)
_REQUEST_KEYS_ALLOWED = frozenset(
    {
        "llm_context",
        "assembly_timestamp_utc_iso",
        "ingest_now_utc_iso",
        "use_deterministic_fake_adapter",
    }
)

OUTCOME_HANDOFF_REQUEST_INVALID = "handoff_request_invalid"


def _parse_iso_utc(s: str) -> datetime:
    raw = str(s).strip()
    if raw.endswith("Z"):
        raw = raw[:-1] + "+00:00"
    dt = datetime.fromisoformat(raw)
    if dt.tzinfo is None:
        raise ValueError("timestamp must be timezone-aware (RFC 3339, Z or offset)")
    return dt.astimezone(timezone.utc)


def parse_handoff_request_json(raw: str) -> Tuple[Mapping[str, Any], datetime, datetime, bool]:
    """
    Parse and validate JSON request body.

    Required keys: ``llm_context`` (object), ``assembly_timestamp_utc_iso``, ``ingest_now_utc_iso``.
    Optional: ``use_deterministic_fake_adapter`` (bool, default ``False``).

    Raises ``ValueError`` with a short message on violation.
    """
    try:
        body = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON: {exc}") from exc
    if not isinstance(body, dict):
        raise ValueError("top-level JSON must be an object")
    keys = frozenset(body.keys())
    if not keys <= _REQUEST_KEYS_ALLOWED:
        extra = sorted(keys - _REQUEST_KEYS_ALLOWED)
        raise ValueError(f"unknown request keys: {extra}")
    if "llm_context" not in body:
        raise ValueError("missing llm_context")
    ctx = body["llm_context"]
    if not isinstance(ctx, Mapping):
        raise ValueError("llm_context must be an object")
    if "assembly_timestamp_utc_iso" not in body or "ingest_now_utc_iso" not in body:
        raise ValueError("assembly_timestamp_utc_iso and ingest_now_utc_iso are required")
    ts_asm = _parse_iso_utc(str(body["assembly_timestamp_utc_iso"]))
    ts_ing = _parse_iso_utc(str(body["ingest_now_utc_iso"]))
    use_fake = body.get("use_deterministic_fake_adapter", False)
    if not isinstance(use_fake, bool):
        raise ValueError("use_deterministic_fake_adapter must be a boolean")
    return ctx, ts_asm, ts_ing, use_fake


def deterministic_fake_adapter_for_handoff_v35() -> Callable[[Mapping[str, Any]], LlmAdapterResult]:
    """
    Local-proof adapter: returns a valid ``llm_candidate`` JSON matching ``location_ref`` in context.
    """

    def _call(ctx: Mapping[str, Any]) -> LlmAdapterSuccess:
        loc = str(ctx.get("location_ref", "")).strip()
        payload = {
            "schema_version": "v3.3.llm_candidate.1",
            "assert_blocked": True,
            "location_ref": loc,
            "confidence": 0.5,
            "ttl_sec": 3600.0,
            "sensor_class": "lidar_occlusion",
        }
        return LlmAdapterSuccess(raw_text=json.dumps(payload))

    return _call


def handoff_result_to_dict(r: SemanticProductionIngestResultV34) -> Dict[str, Any]:
    out: Dict[str, Any] = {"outcome": r.outcome}
    if r.boundary_reason is not None:
        out["boundary_reason"] = r.boundary_reason
    if r.boundary_detail:
        out["boundary_detail"] = r.boundary_detail
    if r.assembly_reason is not None:
        out["assembly_reason"] = r.assembly_reason
    if r.assembly_detail:
        out["assembly_detail"] = r.assembly_detail
    if r.adapter_reason is not None:
        out["adapter_reason"] = r.adapter_reason
    if r.adapter_detail:
        out["adapter_detail"] = r.adapter_detail
    if r.ingest_errors:
        out["ingest_errors"] = list(r.ingest_errors)
    if r.record is not None and r.record.get("belief_id") is not None:
        out["belief_id"] = str(r.record["belief_id"])
    return out


def execute_semantic_production_handoff_v35(
    *,
    llm_context: Mapping[str, Any],
    assembly_timestamp_utc: datetime,
    now_utc_ingest: datetime,
    store: BlockedPassageBeliefStore,
    use_deterministic_fake_adapter: bool = False,
    real_adapter: Optional[Callable[[Mapping[str, Any]], LlmAdapterResult]] = None,
) -> SemanticProductionIngestResultV34:
    """
    Run ``produce_and_ingest_blocked_passage_v34`` with either the deterministic fake adapter or
    ``OpenAiChatCompletionsAdapterV33`` (when ``real_adapter`` is ``None``), or a caller-supplied
    ``real_adapter`` for tests.
    """
    if use_deterministic_fake_adapter:
        adapter_call = deterministic_fake_adapter_for_handoff_v35()
    elif real_adapter is not None:
        adapter_call = real_adapter
    else:
        oa = OpenAiChatCompletionsAdapterV33()
        adapter_call = oa

    return produce_and_ingest_blocked_passage_v34(
        llm_context,
        store=store,
        adapter_call=adapter_call,
        timestamp_utc=assembly_timestamp_utc,
        now_utc_ingest=now_utc_ingest,
    )


def run_handoff_from_json_request(
    json_request: str,
    *,
    store: BlockedPassageBeliefStore,
    real_adapter: Optional[Callable[[Mapping[str, Any]], LlmAdapterResult]] = None,
) -> Dict[str, Any]:
    """
    Parse ``json_request``, execute handoff, return a JSON-serializable dict (includes
    ``outcome`` = ``handoff_request_invalid`` on parse errors).
    """
    try:
        ctx, ts_asm, ts_ing, use_fake = parse_handoff_request_json(json_request)
        result = execute_semantic_production_handoff_v35(
            llm_context=ctx,
            assembly_timestamp_utc=ts_asm,
            now_utc_ingest=ts_ing,
            store=store,
            use_deterministic_fake_adapter=use_fake,
            real_adapter=real_adapter,
        )
        return handoff_result_to_dict(result)
    except ValueError as exc:
        return {
            "outcome": OUTCOME_HANDOFF_REQUEST_INVALID,
            "detail": str(exc),
        }
