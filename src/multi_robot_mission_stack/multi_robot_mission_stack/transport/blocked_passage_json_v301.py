"""
V3.0.1 single-hop transport: JSON object on the wire, unchanged semantic dict shape.

Decoding + ingestion always goes through ``BlockedPassageBeliefStore.ingest`` (validate,
duplicate-ignore, TTL at ingest, store). No alternate receiver semantics.
"""

from __future__ import annotations

import json
from datetime import datetime
from typing import Any, Dict, Mapping

from ..semantic.blocked_passage_v301 import BlockedPassageBeliefStore, IngestResult

# ROS topic default; remap in launch for multi-robot namespaces.
TRANSPORT_TOPIC_DEFAULT = "/semantic/blocked_passage_v301"


def encode_blocked_passage_record_json(record: Mapping[str, Any]) -> str:
    """
    Serialize a ``blocked_passage`` record dict to compact JSON text.

    Caller should pass a schema-valid dict (e.g. from ``build_blocked_passage_record_stub`` or
    a validated store record). This function does not re-validate.
    """
    return json.dumps(record, sort_keys=True, separators=(",", ":"))


def decode_blocked_passage_transport_payload(payload: str) -> Dict[str, Any]:
    """
    Parse JSON; must be a single object. Raises ``ValueError`` on invalid JSON or wrong top type.
    """
    try:
        raw = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise ValueError(f"transport payload is not valid JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise ValueError("transport payload must be a JSON object")
    return raw


def ingest_blocked_passage_transport_payload(
    store: BlockedPassageBeliefStore,
    payload: str,
    *,
    now_utc: datetime,
) -> IngestResult:
    """Decode JSON and ingest via the store (single receiver path)."""
    record = decode_blocked_passage_transport_payload(payload)
    return store.ingest(record, now_utc=now_utc)
