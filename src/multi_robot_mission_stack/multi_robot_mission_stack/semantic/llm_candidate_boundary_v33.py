"""
V3.3 deterministic boundary: validate ``llm_context``, parse strict JSON ``llm_candidate``, map or reject.

No model calls, no ROS. Final record assembly is ``llm_blocked_passage_assembler_v33`` / ``llm_candidate_eval_v33``.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from typing import Any, FrozenSet, List, Mapping, Optional, Tuple, Union

# --- Frozen contract versions ---

LLM_CONTEXT_SCHEMA_VERSION = "v3.3.llm_context.1"
LLM_CANDIDATE_SCHEMA_VERSION = "v3.3.llm_candidate.1"

# --- Enum vocab (frozen) ---

NAV_GOAL_STATUS_VALUES: FrozenSet[str] = frozenset(
    {"unknown", "pending", "active", "succeeded", "failed", "canceled"}
)
PLANNER_STATUS_VALUES: FrozenSet[str] = frozenset(
    {"unknown", "idle", "computing", "failed"}
)

# --- Numeric bounds (frozen for V3.3 boundary) ---

STALL_DURATION_SEC_MIN = 0.0
STALL_DURATION_SEC_MAX = 604800.0
OPERATOR_HINT_MAX_LEN = 200
CONFIDENCE_MIN = 0.0
CONFIDENCE_MAX = 1.0
TTL_SEC_MIN_EXCLUSIVE = 0.0
TTL_SEC_MAX = 86400.0
SENSOR_CLASS_MAX_LEN = 64
RATIONALE_SHORT_MAX_LEN = 280

_CONTEXT_KEYS: FrozenSet[str] = frozenset(
    {
        "schema_version",
        "location_ref",
        "source_robot_id",
        "nav_goal_status",
        "stall_duration_sec",
        "planner_status",
        "lidar_occlusion_proxy",
        "operator_hint",
    }
)

_SENSOR_CLASS_RE = re.compile(r"^[a-z0-9_]+$")

RejectReason = str

# Canonical reason tokens (align with design note §4)
CONTEXT_INVALID = "context_invalid"
CANDIDATE_MALFORMED = "candidate_malformed"
CANDIDATE_MISSING_OR_EXTRA_KEYS = "candidate_missing_or_extra_keys"
CANDIDATE_SCHEMA_MISMATCH = "candidate_schema_mismatch"
CANDIDATE_SEMANTICALLY_INSUFFICIENT = "candidate_semantically_insufficient"
CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH = "candidate_bounds_or_location_mismatch"


@dataclass(frozen=True)
class ValidatedLlmContext:
    """Normalized context after boundary checks (immutable)."""

    location_ref: str
    source_robot_id: str
    nav_goal_status: str
    stall_duration_sec: float
    planner_status: str
    lidar_occlusion_proxy: bool
    operator_hint: str


@dataclass(frozen=True)
class MappedLlmCandidate:
    """
    Fields ready for deterministic final ``blocked_passage`` assembly (plus optional log-only text).
    ``source_robot_id`` comes from context, not the model.
    """

    location_ref: str
    confidence: float
    ttl_sec: float
    sensor_class: str
    source_robot_id: str
    rationale_short: Optional[str]


@dataclass(frozen=True)
class LlmCandidateReject:
    reason: str
    detail: str = ""
    accepted: bool = False


@dataclass(frozen=True)
class LlmCandidateAccept:
    context: ValidatedLlmContext
    mapped: MappedLlmCandidate
    accepted: bool = True


LlmBoundaryResult = Union[LlmCandidateAccept, LlmCandidateReject]


def evaluate_llm_candidate_boundary(
    llm_context: Mapping[str, Any],
    simulated_model_output: str,
) -> LlmBoundaryResult:
    """
    Validate ``llm_context``, parse ``simulated_model_output`` as one strict JSON object,
    validate ``llm_candidate``, return accept with ``MappedLlmCandidate`` or structured reject.
    """
    ctx_err = _validate_llm_context(llm_context)
    if ctx_err is not None:
        reason, detail = ctx_err
        return LlmCandidateReject(reason=reason, detail=detail)

    vctx = _normalize_context(llm_context)

    parsed_err = _parse_strict_json_object(simulated_model_output)
    if isinstance(parsed_err, tuple):
        return LlmCandidateReject(reason=parsed_err[0], detail=parsed_err[1])
    candidate: dict[str, Any] = parsed_err

    return _evaluate_candidate(vctx, candidate)


def _validate_llm_context(obj: Any) -> Optional[Tuple[str, str]]:
    if not isinstance(obj, Mapping):
        return (CONTEXT_INVALID, "llm_context must be a mapping")
    keys = frozenset(obj.keys())
    if keys != _CONTEXT_KEYS:
        missing = sorted(_CONTEXT_KEYS - keys)
        extra = sorted(keys - _CONTEXT_KEYS)
        parts: List[str] = []
        if missing:
            parts.append(f"missing keys: {missing}")
        if extra:
            parts.append(f"extra keys: {extra}")
        return (CONTEXT_INVALID, "; ".join(parts) or "key set mismatch")

    if str(obj.get("schema_version", "")).strip() != LLM_CONTEXT_SCHEMA_VERSION:
        return (CONTEXT_INVALID, f"schema_version must be {LLM_CONTEXT_SCHEMA_VERSION!r}")

    loc = str(obj.get("location_ref", "")).strip()
    if not loc:
        return (CONTEXT_INVALID, "location_ref must be non-empty")

    src = str(obj.get("source_robot_id", "")).strip()
    if not src:
        return (CONTEXT_INVALID, "source_robot_id must be non-empty")

    ngs = str(obj.get("nav_goal_status", "")).strip()
    if ngs not in NAV_GOAL_STATUS_VALUES:
        return (
            CONTEXT_INVALID,
            f"nav_goal_status must be one of {sorted(NAV_GOAL_STATUS_VALUES)}",
        )

    st = obj.get("stall_duration_sec")
    stf, err = _as_float_non_bool(st, "stall_duration_sec")
    if err:
        return (CONTEXT_INVALID, err)
    if stf < STALL_DURATION_SEC_MIN or stf > STALL_DURATION_SEC_MAX:
        return (
            CONTEXT_INVALID,
            f"stall_duration_sec must be in [{STALL_DURATION_SEC_MIN}, {STALL_DURATION_SEC_MAX}]",
        )

    ps = str(obj.get("planner_status", "")).strip()
    if ps not in PLANNER_STATUS_VALUES:
        return (
            CONTEXT_INVALID,
            f"planner_status must be one of {sorted(PLANNER_STATUS_VALUES)}",
        )

    lap = obj.get("lidar_occlusion_proxy")
    if not isinstance(lap, bool):
        return (CONTEXT_INVALID, "lidar_occlusion_proxy must be a boolean")

    hint = obj.get("operator_hint")
    if hint is None:
        return (CONTEXT_INVALID, "operator_hint key required")
    if not isinstance(hint, str):
        return (CONTEXT_INVALID, "operator_hint must be a string")
    ht = hint.strip()
    if len(ht) > OPERATOR_HINT_MAX_LEN:
        return (
            CONTEXT_INVALID,
            f"operator_hint length after strip must be <= {OPERATOR_HINT_MAX_LEN}",
        )

    return None


def _normalize_context(obj: Mapping[str, Any]) -> ValidatedLlmContext:
    hint_raw = obj["operator_hint"]
    hint = hint_raw.strip() if isinstance(hint_raw, str) else ""
    return ValidatedLlmContext(
        location_ref=str(obj["location_ref"]).strip(),
        source_robot_id=str(obj["source_robot_id"]).strip(),
        nav_goal_status=str(obj["nav_goal_status"]).strip(),
        stall_duration_sec=float(obj["stall_duration_sec"]),
        planner_status=str(obj["planner_status"]).strip(),
        lidar_occlusion_proxy=bool(obj["lidar_occlusion_proxy"]),
        operator_hint=hint,
    )


def _parse_strict_json_object(raw: str) -> Union[dict[str, Any], Tuple[str, str]]:
    """
    Exactly one JSON object; no markdown fences; no trailing non-whitespace.
    """
    if not isinstance(raw, str):
        return (CANDIDATE_MALFORMED, "model output must be a string")
    s = raw.strip()
    if not s:
        return (CANDIDATE_MALFORMED, "empty model output")
    if s.startswith("```"):
        return (CANDIDATE_MALFORMED, "markdown fences are not allowed")
    if s[0] != "{":
        return (CANDIDATE_MALFORMED, "model output must start with '{'")
    try:
        decoder = json.JSONDecoder()
        obj, idx = decoder.raw_decode(s)
    except json.JSONDecodeError as exc:
        return (CANDIDATE_MALFORMED, f"invalid JSON: {exc}")
    tail = s[idx:].strip()
    if tail:
        return (CANDIDATE_MALFORMED, f"trailing non-whitespace after JSON: {tail!r}")
    if not isinstance(obj, dict):
        return (CANDIDATE_MALFORMED, "top-level JSON must be an object")
    return obj


def _evaluate_candidate(
    vctx: ValidatedLlmContext,
    c: dict[str, Any],
) -> LlmBoundaryResult:
    keys = frozenset(c.keys())

    if str(c.get("schema_version", "")).strip() != LLM_CANDIDATE_SCHEMA_VERSION:
        return LlmCandidateReject(
            reason=CANDIDATE_SCHEMA_MISMATCH,
            detail=f"schema_version must be {LLM_CANDIDATE_SCHEMA_VERSION!r}",
        )

    ab = c.get("assert_blocked")
    if not isinstance(ab, bool):
        return LlmCandidateReject(
            reason=CANDIDATE_MISSING_OR_EXTRA_KEYS,
            detail="assert_blocked must be a boolean",
        )

    if ab is False:
        allowed_false = frozenset({"schema_version", "assert_blocked"})
        if keys != allowed_false:
            return LlmCandidateReject(
                reason=CANDIDATE_MISSING_OR_EXTRA_KEYS,
                detail=(
                    "when assert_blocked is false, only schema_version and "
                    "assert_blocked are allowed"
                ),
            )
        return LlmCandidateReject(
            reason=CANDIDATE_SEMANTICALLY_INSUFFICIENT,
            detail="assert_blocked is false",
        )

    # assert_blocked true: required key set
    required = frozenset(
        {
            "schema_version",
            "assert_blocked",
            "location_ref",
            "confidence",
            "ttl_sec",
            "sensor_class",
        }
    )
    optional = frozenset({"rationale_short"})
    allowed = required | optional
    if not required.issubset(keys):
        missing = sorted(required - keys)
        return LlmCandidateReject(
            reason=CANDIDATE_MISSING_OR_EXTRA_KEYS,
            detail=f"missing keys: {missing}",
        )
    if not keys.issubset(allowed):
        extra = sorted(keys - allowed)
        return LlmCandidateReject(
            reason=CANDIDATE_MISSING_OR_EXTRA_KEYS,
            detail=f"extra keys: {extra}",
        )

    loc = str(c.get("location_ref", "")).strip()
    if not loc:
        return LlmCandidateReject(
            reason=CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH,
            detail="location_ref must be non-empty when assert_blocked is true",
        )
    if loc != vctx.location_ref:
        return LlmCandidateReject(
            reason=CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH,
            detail="location_ref must exactly equal llm_context.location_ref",
        )

    conf, err = _as_float_non_bool(c.get("confidence"), "confidence")
    if err:
        return LlmCandidateReject(reason=CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH, detail=err)
    if conf < CONFIDENCE_MIN or conf > CONFIDENCE_MAX:
        return LlmCandidateReject(
            reason=CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH,
            detail=f"confidence must be in [{CONFIDENCE_MIN}, {CONFIDENCE_MAX}]",
        )

    ttl, err = _as_float_non_bool(c.get("ttl_sec"), "ttl_sec")
    if err:
        return LlmCandidateReject(reason=CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH, detail=err)
    if ttl <= TTL_SEC_MIN_EXCLUSIVE or ttl > TTL_SEC_MAX:
        return LlmCandidateReject(
            reason=CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH,
            detail=f"ttl_sec must be in ({TTL_SEC_MIN_EXCLUSIVE}, {TTL_SEC_MAX}]",
        )

    sc = str(c.get("sensor_class", "")).strip()
    if not sc or len(sc) > SENSOR_CLASS_MAX_LEN:
        return LlmCandidateReject(
            reason=CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH,
            detail="sensor_class must be non-empty and within max length",
        )
    if not _SENSOR_CLASS_RE.fullmatch(sc):
        return LlmCandidateReject(
            reason=CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH,
            detail="sensor_class must match [a-z0-9_]+",
        )

    rationale: Optional[str] = None
    if "rationale_short" in c:
        rs = c["rationale_short"]
        if rs is None:
            return LlmCandidateReject(
                reason=CANDIDATE_MISSING_OR_EXTRA_KEYS,
                detail="rationale_short must not be null",
            )
        if not isinstance(rs, str):
            return LlmCandidateReject(
                reason=CANDIDATE_MISSING_OR_EXTRA_KEYS,
                detail="rationale_short must be a string",
            )
        rationale = rs.strip()
        if len(rationale) > RATIONALE_SHORT_MAX_LEN:
            return LlmCandidateReject(
                reason=CANDIDATE_BOUNDS_OR_LOCATION_MISMATCH,
                detail=f"rationale_short length after strip must be <= {RATIONALE_SHORT_MAX_LEN}",
            )
        if not rationale:
            rationale = None

    mapped = MappedLlmCandidate(
        location_ref=loc,
        confidence=conf,
        ttl_sec=ttl,
        sensor_class=sc,
        source_robot_id=vctx.source_robot_id,
        rationale_short=rationale,
    )
    return LlmCandidateAccept(context=vctx, mapped=mapped)


def _as_float_non_bool(value: Any, field: str) -> Tuple[float, Optional[str]]:
    if isinstance(value, bool):
        return 0.0, f"{field} must be a number, not bool"
    if isinstance(value, int):
        return float(value), None
    if isinstance(value, float):
        return value, None
    return 0.0, f"{field} must be a number"

