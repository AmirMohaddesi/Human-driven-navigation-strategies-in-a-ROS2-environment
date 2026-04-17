"""
V8.5-style deterministic assembly: small candidate → final ``degraded_passage`` V8.1 dict.

No ROS, no store, no policy. Used by P3 visibility tests and bridge demos.
"""

from __future__ import annotations

import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Dict, Optional

from .degraded_passage_v81 import (
    FACT_TYPE_DEGRADED_PASSAGE,
    SCHEMA_VERSION,
    VERIFICATION_STATUS_UNVERIFIED,
    validate_degraded_passage_record,
)


def _timestamp_utc_to_rfc3339_z(ts: datetime) -> str:
    if ts.tzinfo is None:
        raise ValueError("timestamp_utc must be timezone-aware (RFC 3339 UTC or offset)")
    u = ts.astimezone(timezone.utc)
    s = u.isoformat()
    if s.endswith("+00:00"):
        s = s[:-6] + "Z"
    return s


@dataclass(frozen=True)
class DegradedPassageAssemblyCandidate:
    location_ref: str
    source_robot_id: str
    confidence: float
    ttl_sec: float
    degradation_class: str
    recommended_speed_factor: float
    sensor_class: str


@dataclass(frozen=True)
class DegradedPassageAssembledAccept:
    record: Dict[str, Any]


def assemble_degraded_passage_record_from_candidate(
    candidate: DegradedPassageAssemblyCandidate,
    *,
    timestamp_utc: datetime,
    belief_id: Optional[str] = None,
    observation_id: Optional[str] = None,
) -> DegradedPassageAssembledAccept:
    bid = str(uuid.uuid4()) if belief_id is None else str(belief_id).strip()
    oid = str(uuid.uuid4()) if observation_id is None else str(observation_id).strip()

    record: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "belief_id": bid,
        "fact_type": FACT_TYPE_DEGRADED_PASSAGE,
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
        "degradation_class": str(candidate.degradation_class).strip(),
        "recommended_speed_factor": float(candidate.recommended_speed_factor),
    }

    ok, errs = validate_degraded_passage_record(record)
    if not ok:
        raise ValueError("assembled degraded_passage failed validation: " + "; ".join(errs))

    return DegradedPassageAssembledAccept(record=record)
