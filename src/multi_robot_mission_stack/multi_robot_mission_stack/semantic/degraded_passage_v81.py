"""
V8.1/V8.2 ``degraded_passage`` local contract validation (no ROS, no transport, no policy hooks).
"""

from __future__ import annotations

import uuid
from dataclasses import dataclass
from datetime import timedelta
from datetime import datetime, timezone
from typing import Any, Dict, FrozenSet, List, Mapping, Optional, Tuple

SCHEMA_VERSION = "v8.1"
FACT_TYPE_DEGRADED_PASSAGE = "degraded_passage"
VERIFICATION_STATUS_UNVERIFIED = "unverified"
TTL_SKEW_ALLOWANCE_SEC = 0.5

DEGRADATION_CLASS_ALLOWED: FrozenSet[str] = frozenset(
    {
        "slow_zone",
        "narrow_clearance",
        "intermittent_obstacle",
        "surface_uncertain",
    }
)

_REQUIRED_KEYS: FrozenSet[str] = frozenset(
    {
        "schema_version",
        "belief_id",
        "fact_type",
        "source_robot_id",
        "timestamp_utc",
        "confidence",
        "location_ref",
        "provenance",
        "ttl_sec",
        "verification_status",
        "degradation_class",
        "recommended_speed_factor",
    }
)


def _is_uuid_v4(s: str) -> bool:
    try:
        u = uuid.UUID(str(s).strip())
    except (ValueError, TypeError, AttributeError):
        return False
    return u.version == 4


def _parse_rfc3339_utc(s: str) -> datetime:
    raw = str(s).strip()
    if raw.endswith("Z"):
        raw = raw[:-1] + "+00:00"
    dt = datetime.fromisoformat(raw)
    if dt.tzinfo is None:
        raise ValueError("timestamp_utc must be timezone-aware (RFC 3339 with Z or offset)")
    return dt.astimezone(timezone.utc)


def _float_in_range(value: Any, *, lo: float, hi: float, lo_inclusive: bool) -> bool:
    if isinstance(value, bool):
        return False
    if isinstance(value, (int, float)):
        v = float(value)
    else:
        return False
    if lo_inclusive:
        if v < lo:
            return False
    elif v <= lo:
        return False
    return v <= hi


def validate_degraded_passage_record(record: Mapping[str, Any]) -> Tuple[bool, List[str]]:
    """
    Validate a candidate ``degraded_passage`` dict against the V8.1 frozen contract.

    Returns (ok, errors) where errors is non-empty when not ok.
    """
    errors: List[str] = []

    if not isinstance(record, Mapping):
        return False, ["record must be a mapping"]

    keys = frozenset(record.keys())
    if keys != _REQUIRED_KEYS:
        missing = sorted(_REQUIRED_KEYS - keys)
        extra = sorted(keys - _REQUIRED_KEYS)
        if missing:
            errors.append(f"missing fields: {missing}")
        if extra:
            errors.append(f"unknown fields: {extra}")

    def req(key: str) -> Any:
        if key not in record:
            return None
        return record[key]

    sv = req("schema_version")
    if sv is not None and sv != SCHEMA_VERSION:
        errors.append(f"schema_version must be exactly {SCHEMA_VERSION!r}")

    bid = req("belief_id")
    if bid is not None and (not isinstance(bid, str) or not _is_uuid_v4(bid)):
        errors.append("belief_id must be a UUID v4 string")

    ft = req("fact_type")
    if ft is not None and ft != FACT_TYPE_DEGRADED_PASSAGE:
        errors.append(f"fact_type must be exactly {FACT_TYPE_DEGRADED_PASSAGE!r}")

    src = req("source_robot_id")
    if src is not None and (not isinstance(src, str) or not src.strip()):
        errors.append("source_robot_id must be a non-empty string")

    ts = req("timestamp_utc")
    if ts is not None:
        if not isinstance(ts, str):
            errors.append("timestamp_utc must be a string (RFC 3339)")
        else:
            try:
                _parse_rfc3339_utc(ts)
            except (ValueError, TypeError) as exc:
                errors.append(f"timestamp_utc parse error: {exc}")

    conf = req("confidence")
    if conf is not None and not _float_in_range(conf, lo=0.0, hi=1.0, lo_inclusive=True):
        errors.append("confidence must be a float in [0.0, 1.0] (not bool)")

    loc = req("location_ref")
    if loc is not None and (not isinstance(loc, str) or not loc.strip()):
        errors.append("location_ref must be a non-empty string")

    prov = req("provenance")
    if prov is not None:
        if not isinstance(prov, Mapping):
            errors.append("provenance must be an object (mapping)")
        else:
            sc = prov.get("sensor_class")
            oid = prov.get("observation_id")
            if not isinstance(sc, str) or not sc.strip():
                errors.append("provenance.sensor_class is required and must be a non-empty string")
            if not isinstance(oid, str) or not _is_uuid_v4(oid):
                errors.append("provenance.observation_id is required and must be a UUID v4 string")

    ttl = req("ttl_sec")
    if ttl is not None and not _float_in_range(ttl, lo=0.0, hi=float("inf"), lo_inclusive=False):
        errors.append("ttl_sec must be a number > 0.0")

    vs = req("verification_status")
    if vs is not None and vs != VERIFICATION_STATUS_UNVERIFIED:
        errors.append(f"verification_status must be exactly {VERIFICATION_STATUS_UNVERIFIED!r}")

    dclass = req("degradation_class")
    if dclass is not None and dclass not in DEGRADATION_CLASS_ALLOWED:
        errors.append(f"degradation_class must be one of {sorted(DEGRADATION_CLASS_ALLOWED)}")

    sf = req("recommended_speed_factor")
    if sf is not None and not _float_in_range(sf, lo=0.0, hi=1.0, lo_inclusive=False):
        errors.append("recommended_speed_factor must be a float in (0.0, 1.0] (not bool)")

    return (len(errors) == 0, errors)


def _active_deadline_utc(assertion_end: datetime, ttl_sec: float) -> datetime:
    return assertion_end + timedelta(seconds=float(ttl_sec) + TTL_SKEW_ALLOWANCE_SEC)


def is_degraded_passage_active(
    record: Mapping[str, Any],
    *,
    now_utc: datetime,
) -> Tuple[bool, Optional[str]]:
    """
    Given a validated degraded_passage record, return (active, error_reason).
    """
    ok, errs = validate_degraded_passage_record(record)
    if not ok:
        return False, "; ".join(errs)
    try:
        start = _parse_rfc3339_utc(str(record["timestamp_utc"]))
        deadline = _active_deadline_utc(start, float(record["ttl_sec"]))
        now = now_utc.astimezone(timezone.utc)
        return (now <= deadline, None)
    except (KeyError, ValueError, TypeError) as exc:
        return False, str(exc)


@dataclass(frozen=True)
class IngestResult:
    stored: bool
    duplicate_ignored: bool
    rejected: bool
    errors: Tuple[str, ...]


@dataclass(frozen=True)
class ActiveDegradedQueryResult:
    has_active: bool
    active_belief_ids: Tuple[str, ...]


class DegradedPassageBeliefStore:
    """
    In-memory fact-specific store for degraded_passage records.
    """

    def __init__(self, *, allowed_source_robot_ids: Optional[FrozenSet[str]] = None) -> None:
        self._allowed = allowed_source_robot_ids
        self._by_id: Dict[str, Dict[str, Any]] = {}

    def ingest(
        self,
        record: Mapping[str, Any],
        *,
        now_utc: datetime,
    ) -> IngestResult:
        ok, errs = validate_degraded_passage_record(record)
        if not ok:
            return IngestResult(
                stored=False,
                duplicate_ignored=False,
                rejected=True,
                errors=tuple(errs),
            )

        bid = str(record["belief_id"]).strip()
        if bid in self._by_id:
            return IngestResult(
                stored=False,
                duplicate_ignored=True,
                rejected=False,
                errors=(),
            )

        src = str(record["source_robot_id"]).strip()
        if self._allowed is not None and src not in self._allowed:
            return IngestResult(
                stored=False,
                duplicate_ignored=False,
                rejected=True,
                errors=(f"source_robot_id not in allowlist: {src!r}",),
            )

        active, err = is_degraded_passage_active(record, now_utc=now_utc)
        if err:
            return IngestResult(
                stored=False,
                duplicate_ignored=False,
                rejected=True,
                errors=(err,),
            )
        if not active:
            return IngestResult(
                stored=False,
                duplicate_ignored=False,
                rejected=True,
                errors=("record already expired at ingest time (TTL + skew)",),
            )

        self._by_id[bid] = dict(record)
        return IngestResult(
            stored=True,
            duplicate_ignored=False,
            rejected=False,
            errors=(),
        )

    def has_active_degraded_passage(
        self,
        location_name: str,
        *,
        now_utc: datetime,
    ) -> ActiveDegradedQueryResult:
        want = str(location_name).strip()
        matches: List[str] = []
        for bid, rec in self._by_id.items():
            loc = str(rec.get("location_ref", "")).strip()
            if loc != want:
                continue
            active, _ = is_degraded_passage_active(rec, now_utc=now_utc)
            if active:
                matches.append(bid)
        t = tuple(sorted(matches))
        return ActiveDegradedQueryResult(has_active=len(t) > 0, active_belief_ids=t)

    def get_record(self, belief_id: str) -> Optional[Dict[str, Any]]:
        rec = self._by_id.get(str(belief_id).strip())
        return dict(rec) if rec is not None else None

    def __len__(self) -> int:
        return len(self._by_id)
