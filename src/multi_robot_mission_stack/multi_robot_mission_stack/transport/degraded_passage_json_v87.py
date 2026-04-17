"""
V8.7 transport helper: JSON text for ``degraded_passage`` wire payloads (single object).

Same compact encoding style as ``blocked_passage_json_v301.encode_blocked_passage_record_json``.
"""

from __future__ import annotations

import json
from typing import Any, Mapping


def encode_degraded_passage_record_json(record: Mapping[str, Any]) -> str:
    """Serialize a validated ``degraded_passage`` record dict to compact JSON text."""
    return json.dumps(record, sort_keys=True, separators=(",", ":"))
