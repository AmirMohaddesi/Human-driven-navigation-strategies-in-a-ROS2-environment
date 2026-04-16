"""
V11.5 deterministic local candidate → full ``shared_space_caution`` record assembly (no ROS).

``belief_id``, ``timestamp_utc``, ``fact_type``, ``schema_version``, ``verification_status``, and
``provenance.observation_id`` are code-owned. The candidate carries only bounded producer fields.
Final acceptance is always ``validate_shared_space_caution_record``.
"""

from __future__ import annotations

import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple, Union

from .shared_space_caution_v111 import (
    CAUTION_CLASS_ALLOWED,
    FACT_TYPE_SHARED_SPACE_CAUTION,
    SCHEMA_VERSION,
    VERIFICATION_STATUS_UNVERIFIED,
    validate_shared_space_caution_record,
)

FINAL_SCHEMA_REJECT = "final_schema_reject"
CANDIDATE_INVALID = "candidate_invalid"


def _timestamp_utc_to_rfc3339_z(ts: datetime) -> str:
    if ts.tzinfo is None:
        raise ValueError("timestamp_utc must be timezone-aware (RFC 3339 UTC or offset)")
    u = ts.astimezone(timezone.utc)
    s = u.isoformat()
    if s.endswith("+00:00"):
        s = s[:-6] + "Z"
    return s


@dataclass(frozen=True)
class SharedSpaceCautionAssemblyCandidate:
    """
    Narrow producer-facing surface: everything except code-owned identity, timestamps, and
    ``verification_status``.
    """

    location_ref: str
    source_robot_id: str
    confidence: float
    ttl_sec: float
    caution_class: str
    sensor_class: str


@dataclass(frozen=True)
class SharedSpaceCautionAssembledReject:
    reason: str
    detail: str
    accepted: bool = False


@dataclass(frozen=True)
class SharedSpaceCautionAssembledAccept:
    record: Dict[str, Any]
    accepted: bool = True


SharedSpaceCautionAssemblyResult = Union[
    SharedSpaceCautionAssembledAccept, SharedSpaceCautionAssembledReject
]


def _validate_candidate(candidate: SharedSpaceCautionAssemblyCandidate) -> Tuple[bool, List[str]]:
    errors: List[str] = []
    loc = str(candidate.location_ref).strip()
    if not loc:
        errors.append("location_ref must be non-empty")
    src = str(candidate.source_robot_id).strip()
    if not src:
        errors.append("source_robot_id must be non-empty")
    sc = str(candidate.sensor_class).strip()
    if not sc:
        errors.append("sensor_class must be non-empty")

    if isinstance(candidate.confidence, bool) or not isinstance(candidate.confidence, (int, float)):
        errors.append("confidence must be a number (not bool)")
    elif not (0.0 <= float(candidate.confidence) <= 1.0):
        errors.append("confidence must be in [0.0, 1.0]")

    if isinstance(candidate.ttl_sec, bool) or not isinstance(candidate.ttl_sec, (int, float)):
        errors.append("ttl_sec must be a number (not bool)")
    elif float(candidate.ttl_sec) <= 0.0:
        errors.append("ttl_sec must be > 0.0")

    cc = str(candidate.caution_class).strip()
    if cc not in CAUTION_CLASS_ALLOWED:
        errors.append(f"caution_class must be one of {sorted(CAUTION_CLASS_ALLOWED)}")

    return (len(errors) == 0, errors)


def assemble_shared_space_caution_record_from_candidate(
    candidate: SharedSpaceCautionAssemblyCandidate,
    *,
    timestamp_utc: datetime,
    belief_id: Optional[str] = None,
    observation_id: Optional[str] = None,
) -> SharedSpaceCautionAssemblyResult:
    """
    Build a full V11.1 ``shared_space_caution`` mapping from a validated candidate + explicit clock.

    Omitted ``belief_id`` / ``observation_id`` → fresh UUID v4 in code. Invalid explicit ids yield
    ``final_schema_reject`` after assembly (validator-owned).
    """
    ok, cerrs = _validate_candidate(candidate)
    if not ok:
        return SharedSpaceCautionAssembledReject(
            reason=CANDIDATE_INVALID,
            detail="; ".join(cerrs),
        )

    bid = str(uuid.uuid4()) if belief_id is None else str(belief_id).strip()
    oid = str(uuid.uuid4()) if observation_id is None else str(observation_id).strip()

    record: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "belief_id": bid,
        "fact_type": FACT_TYPE_SHARED_SPACE_CAUTION,
        "source_robot_id": str(candidate.source_robot_id).strip(),
        "timestamp_utc": _timestamp_utc_to_rfc3339_z(timestamp_utc),
        "confidence": float(candidate.confidence),
        "location_ref": str(candidate.location_ref).strip(),
        "provenance": {
            "sensor_class": str(candidate.sensor_class).strip(),
            "observation_id": oid,
        },
        "ttl_sec": float(candidate.ttl_sec),
        "verification_status": VERIFICATION_STATUS_UNVERIFIED,
        "caution_class": str(candidate.caution_class).strip(),
    }

    vok, verrs = validate_shared_space_caution_record(record)
    if not vok:
        return SharedSpaceCautionAssembledReject(
            reason=FINAL_SCHEMA_REJECT,
            detail="; ".join(verrs),
        )
    return SharedSpaceCautionAssembledAccept(record=record)
