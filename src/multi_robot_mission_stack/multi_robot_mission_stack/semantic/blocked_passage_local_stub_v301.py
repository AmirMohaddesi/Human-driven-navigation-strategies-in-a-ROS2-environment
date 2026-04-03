"""
Local, deterministic ``blocked_passage`` record construction for V3.0.1 (no transport, no LLM).

Builds a dict that satisfies ``validate_blocked_passage_record`` from explicit caller inputs.
Raises ``ValueError`` with schema validator messages when the result would be invalid.
"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Dict

from .blocked_passage_v301 import (
    FACT_TYPE_BLOCKED_PASSAGE,
    SCHEMA_VERSION,
    VERIFICATION_STATUS_UNVERIFIED,
    validate_blocked_passage_record,
)


def _timestamp_utc_to_rfc3339_z(ts: datetime) -> str:
    if ts.tzinfo is None:
        raise ValueError("timestamp_utc must be timezone-aware (RFC 3339 UTC or offset)")
    u = ts.astimezone(timezone.utc)
    s = u.isoformat()
    if s.endswith("+00:00"):
        s = s[:-6] + "Z"
    return s


def build_blocked_passage_record_stub(
    *,
    belief_id: str,
    source_robot_id: str,
    location_ref: str,
    confidence: float,
    timestamp_utc: datetime,
    ttl_sec: float,
    sensor_class: str,
    observation_id: str,
) -> Dict[str, Any]:
    """
    Assemble a V3.0.1 ``blocked_passage`` mapping. ``belief_id`` and ``observation_id`` must be
    UUID v4 strings; ``timestamp_utc`` must be timezone-aware. No wall-clock defaults.
    """
    record: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "belief_id": str(belief_id).strip(),
        "fact_type": FACT_TYPE_BLOCKED_PASSAGE,
        "source_robot_id": str(source_robot_id).strip(),
        "timestamp_utc": _timestamp_utc_to_rfc3339_z(timestamp_utc),
        "confidence": float(confidence),
        "location_ref": str(location_ref).strip(),
        "provenance": {
            "sensor_class": str(sensor_class).strip(),
            "observation_id": str(observation_id).strip(),
        },
        "ttl_sec": float(ttl_sec),
        "verification_status": VERIFICATION_STATUS_UNVERIFIED,
    }
    ok, errs = validate_blocked_passage_record(record)
    if not ok:
        raise ValueError("; ".join(errs))
    return record
