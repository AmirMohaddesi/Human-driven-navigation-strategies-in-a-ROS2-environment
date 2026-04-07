"""
V3.3 deterministic assembly: ``LlmCandidateAccept`` → final ``blocked_passage`` dict + schema gate.

No model, no ROS, no store/transport. ``timestamp_utc`` is required (no implicit wall clock).
"""

from __future__ import annotations

import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Dict, Optional, Union

from .blocked_passage_v301 import (
    FACT_TYPE_BLOCKED_PASSAGE,
    SCHEMA_VERSION,
    VERIFICATION_STATUS_UNVERIFIED,
    validate_blocked_passage_record,
)
from .llm_candidate_boundary_v33 import LlmCandidateAccept

# §4 design note — post-assembly validator failure
FINAL_SCHEMA_REJECT = "final_schema_reject"


def _timestamp_utc_to_rfc3339_z(ts: datetime) -> str:
    if ts.tzinfo is None:
        raise ValueError("timestamp_utc must be timezone-aware (RFC 3339 UTC or offset)")
    u = ts.astimezone(timezone.utc)
    s = u.isoformat()
    if s.endswith("+00:00"):
        s = s[:-6] + "Z"
    return s


@dataclass(frozen=True)
class LlmAssembledReject:
    reason: str
    detail: str
    accepted: bool = False


@dataclass(frozen=True)
class LlmAssembledAccept:
    record: Dict[str, Any]
    accepted: bool = True


LlmAssemblyResult = Union[LlmAssembledAccept, LlmAssembledReject]


def assemble_blocked_passage_from_v33_accept(
    accept: LlmCandidateAccept,
    *,
    timestamp_utc: datetime,
    belief_id: Optional[str] = None,
    observation_id: Optional[str] = None,
) -> LlmAssemblyResult:
    """
    Build a V3.0.1 ``blocked_passage`` mapping from a boundary-accepted V3.3 candidate.

    ``belief_id`` and ``provenance.observation_id`` are UUID v4 strings generated in code when
    omitted. Callers may pass explicit strings for deterministic tests; invalid values yield
    ``final_schema_reject`` with validator messages in ``detail`` only.
    """
    mapped = accept.mapped
    bid = str(uuid.uuid4()) if belief_id is None else str(belief_id).strip()
    oid = str(uuid.uuid4()) if observation_id is None else str(observation_id).strip()

    record: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "belief_id": bid,
        "fact_type": FACT_TYPE_BLOCKED_PASSAGE,
        "source_robot_id": mapped.source_robot_id,
        "timestamp_utc": _timestamp_utc_to_rfc3339_z(timestamp_utc),
        "confidence": float(mapped.confidence),
        "location_ref": mapped.location_ref,
        "provenance": {
            "sensor_class": mapped.sensor_class,
            "observation_id": oid,
        },
        "ttl_sec": float(mapped.ttl_sec),
        "verification_status": VERIFICATION_STATUS_UNVERIFIED,
    }

    ok, errs = validate_blocked_passage_record(record)
    if not ok:
        return LlmAssembledReject(
            reason=FINAL_SCHEMA_REJECT,
            detail="; ".join(errs),
        )
    return LlmAssembledAccept(record=record)
