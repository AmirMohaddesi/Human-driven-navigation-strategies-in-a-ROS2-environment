"""
V5.1 — machine-readable classification for navigate result dicts (orchestration only).

Detects the frozen advisory ``blocked_passage`` peer-belief outcome produced by
``MissionTools._blocked_by_peer_belief_response`` / ``make_blocked_by_peer_belief_outcome``.
Does not apply to bridge-normalized dicts that omit ``outcome`` / ``reason_code``.
"""

from __future__ import annotations

from typing import Any, Mapping, Optional

from ..semantic.blocked_passage_v301 import (
    BLOCKED_OUTCOME_VALUE,
    BLOCKED_REASON_CODE,
)

# Single allowlisted token for step/leg records (additive field only).
NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE = "advisory_blocked_passage"


def navigate_failure_kind(nav: Mapping[str, Any]) -> Optional[str]:
    """
    Return ``NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE`` iff ``nav`` matches the frozen
    V3.0.1 advisory blocked outcome shape; else ``None``.
    """
    if not isinstance(nav, dict):
        return None
    if str(nav.get("outcome", "")).strip() != BLOCKED_OUTCOME_VALUE:
        return None
    if str(nav.get("reason_code", "")).strip() != BLOCKED_REASON_CODE:
        return None
    return NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE
