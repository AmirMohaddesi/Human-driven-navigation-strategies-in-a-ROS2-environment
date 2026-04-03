"""
V3.0.1 ``blocked_passage`` belief validation and active-belief evaluation (no ROS, no transport).

Frozen implementation decisions (manager-approved for this slice):

1) **Clock source**
   All TTL / active checks use an explicit **UTC** instant passed as ``now_utc`` (timezone-aware,
   normalized to UTC). Callers **must** supply ``now_utc`` in production and tests. The module
   **does not** call ``datetime.now()`` internally except via helpers that require the caller to pass
   time in — the store's public methods take ``now_utc: datetime`` as a **required** parameter for
   ``ingest`` and ``has_active_blocked_passage`` so evaluation is deterministic and testable.

   (If we allowed optional None, we'd hide clock source; requirement is explicit — so **required**
   ``now_utc`` everywhere a decision depends on time.)

2) **Skew allowance**
   Fixed constant ``TTL_SKEW_ALLOWANCE_SEC = 0.5`` seconds, added to ``ttl_sec`` when computing the
   active deadline: a fact is **active** iff ``now_utc <= assertion_end + ttl_sec + skew`` (see
   ``_active_deadline_utc``).

3) **Provenance**
   ``provenance`` is **required** and must be a ``dict`` with **both** keys present and valid:
   - ``sensor_class``: non-empty string (no max length enforced in v3.0.1).
   - ``observation_id``: string, UUID v4 (same format rules as ``belief_id``).

4) **Duplicates**
   Ingestion keyed by ``belief_id``. If ``belief_id`` already exists in the store, the second ingest
   is **ignored** (no overwrite, no error): ``IngestResult.duplicate_ignored=True`` and
   ``IngestResult.stored=False``.

5) **Blocked outcome (no-fallback case)**
   Structured dict shape frozen for mission-layer returns — see ``make_blocked_by_peer_belief_outcome``
   and constants ``BLOCKED_OUTCOME_*`` below.
"""

from __future__ import annotations

import uuid
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Any, Dict, FrozenSet, List, Mapping, Optional, Tuple

# --- Frozen constants ---

SCHEMA_VERSION = "v3.0.1"
FACT_TYPE_BLOCKED_PASSAGE = "blocked_passage"
VERIFICATION_STATUS_UNVERIFIED = "unverified"

TTL_SKEW_ALLOWANCE_SEC = 0.5

BLOCKED_OUTCOME_SCHEMA_VERSION = "v3.0.1"
BLOCKED_OUTCOME_VALUE = "navigation_target_blocked"
BLOCKED_REASON_CODE = "blocked_passage_peer_belief"


def make_blocked_by_peer_belief_outcome(
    *,
    requested_location_name: str,
    active_belief_ids: Tuple[str, ...],
) -> Dict[str, Any]:
    """
    Structured outcome when robot B must not navigate to ``requested_location_name`` and no
    fallback target is configured. Mission-layer JSON / dict contract for V3.0.1.
    """
    return {
        "outcome": BLOCKED_OUTCOME_VALUE,
        "schema_version": BLOCKED_OUTCOME_SCHEMA_VERSION,
        "reason_code": BLOCKED_REASON_CODE,
        "requested_location_name": str(requested_location_name).strip(),
        "active_belief_ids": list(active_belief_ids),
    }


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


def _confidence_ok(c: Any) -> bool:
    if isinstance(c, bool):
        return False
    if isinstance(c, int):
        c = float(c)
    if not isinstance(c, float):
        return False
    return 0.0 <= c <= 1.0


def validate_blocked_passage_record(record: Mapping[str, Any]) -> Tuple[bool, List[str]]:
    """
    Validate a candidate ``blocked_passage`` dict against the V3.0.1 frozen schema.

    Returns (ok, errors) where errors is a non-empty list of human-readable reasons if not ok.
    """
    errors: List[str] = []

    if not isinstance(record, Mapping):
        return False, ["record must be a mapping"]

    def req(key: str) -> Any:
        if key not in record:
            errors.append(f"missing field: {key}")
            return None
        return record[key]

    sv = req("schema_version")
    if sv is not None and sv != SCHEMA_VERSION:
        errors.append(f"schema_version must be exactly {SCHEMA_VERSION!r}")

    bid = req("belief_id")
    if bid is not None:
        if not isinstance(bid, str) or not _is_uuid_v4(bid):
            errors.append("belief_id must be a UUID v4 string")

    ft = req("fact_type")
    if ft is not None and ft != FACT_TYPE_BLOCKED_PASSAGE:
        errors.append(f"fact_type must be exactly {FACT_TYPE_BLOCKED_PASSAGE!r}")

    src = req("source_robot_id")
    if src is not None:
        if not isinstance(src, str) or not str(src).strip():
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
    if conf is not None and not _confidence_ok(conf):
        errors.append("confidence must be a float in [0.0, 1.0] (not bool)")

    loc = req("location_ref")
    if loc is not None:
        if not isinstance(loc, str) or not str(loc).strip():
            errors.append("location_ref must be a non-empty string")

    prov = req("provenance")
    if prov is not None:
        if not isinstance(prov, Mapping):
            errors.append("provenance must be an object (mapping)")
        else:
            sc = prov.get("sensor_class")
            oid = prov.get("observation_id")
            if not isinstance(sc, str) or not str(sc).strip():
                errors.append("provenance.sensor_class is required and must be a non-empty string")
            if not isinstance(oid, str) or not _is_uuid_v4(oid):
                errors.append("provenance.observation_id is required and must be a UUID v4 string")

    ttl = req("ttl_sec")
    if ttl is not None:
        if isinstance(ttl, bool) or not isinstance(ttl, (int, float)):
            errors.append("ttl_sec must be a number")
        elif float(ttl) <= 0.0:
            errors.append("ttl_sec must be > 0.0")

    vs = req("verification_status")
    if vs is not None and vs != VERIFICATION_STATUS_UNVERIFIED:
        errors.append(f"verification_status must be exactly {VERIFICATION_STATUS_UNVERIFIED!r}")

    return (len(errors) == 0, errors)


def _active_deadline_utc(assertion_end: datetime, ttl_sec: float) -> datetime:
    return assertion_end + timedelta(seconds=float(ttl_sec) + TTL_SKEW_ALLOWANCE_SEC)


def is_blocked_passage_active(
    record: Mapping[str, Any],
    *,
    now_utc: datetime,
) -> Tuple[bool, Optional[str]]:
    """
    Given a **validated** record dict, return (active, error_reason).

    If the record is structurally invalid, returns (False, reason) without raising.
    """
    ok, errs = validate_blocked_passage_record(record)
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
class ActiveBlockedQueryResult:
    """Result of ``has_active_blocked_passage`` — any active ``blocked_passage`` for this name."""

    has_active: bool
    active_belief_ids: Tuple[str, ...]


class BlockedPassageBeliefStore:
    """
    In-memory store: validate, dedupe by ``belief_id``, TTL-evaluated queries only.

    ``allowed_source_robot_ids``: if not ``None``, only records whose ``source_robot_id`` is in this
    set are ingested; others are rejected with an explicit error.
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
        """
        Validate ``record``. If invalid → rejected. If duplicate ``belief_id`` → ignored.
        If already expired at ``now_utc`` → rejected (not stored for live policy).
        If ``allowed_source_robot_ids`` is set and ``source_robot_id`` not allowed → rejected.
        """
        ok, errs = validate_blocked_passage_record(record)
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

        active, err = is_blocked_passage_active(record, now_utc=now_utc)
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

        rec = dict(record)
        self._by_id[bid] = rec
        return IngestResult(
            stored=True,
            duplicate_ignored=False,
            rejected=False,
            errors=(),
        )

    def has_active_blocked_passage(
        self,
        location_name: str,
        *,
        now_utc: datetime,
    ) -> ActiveBlockedQueryResult:
        """
        True iff any **stored** belief has ``location_ref`` string-equal to ``location_name`` (strip)
        and is **active** at ``now_utc`` per TTL + skew rule.
        """
        want = str(location_name).strip()
        matches: List[str] = []
        for bid, rec in self._by_id.items():
            loc = str(rec.get("location_ref", "")).strip()
            if loc != want:
                continue
            active, _ = is_blocked_passage_active(rec, now_utc=now_utc)
            if active:
                matches.append(bid)
        t = tuple(sorted(matches))
        return ActiveBlockedQueryResult(has_active=len(t) > 0, active_belief_ids=t)

    def get_record(self, belief_id: str) -> Optional[Dict[str, Any]]:
        """Return a copy of the stored record, if any."""
        r = self._by_id.get(str(belief_id).strip())
        return dict(r) if r is not None else None

    def __len__(self) -> int:
        return len(self._by_id)
