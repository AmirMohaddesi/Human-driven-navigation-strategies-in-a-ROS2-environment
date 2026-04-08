"""
V5.1 + V5.2 — machine-readable classification for navigate result dicts (orchestration only).

1) **In-process / full dict:** ``outcome`` + ``reason_code`` match frozen V3.0.1 advisory block
   (``MissionTools._blocked_by_peer_belief_response`` / ``make_blocked_by_peer_belief_outcome``).

2) **Bridge wire-normalized dict (V5.2):** ``NavigateToNamedLocation`` carries no ``outcome`` /
   ``reason_code``; the bridge and client surface only ``status``, ``message``, ``goal_id``,
   ``nav_status``. The same advisory block is recognized via the frozen failure line plus
   failed status and empty goal (``MissionClient`` / ``MissionTools._normalize_goal_response`` shape).
"""

from __future__ import annotations

from typing import Any, Mapping, Optional

from ..semantic.blocked_passage_v301 import (
    BLOCKED_OUTCOME_VALUE,
    BLOCKED_PEER_BELIEF_FAILURE_MESSAGE,
    BLOCKED_REASON_CODE,
)

# Single allowlisted token for step/leg records (additive field only).
NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE = "advisory_blocked_passage"


def _goal_id_empty(nav: Mapping[str, Any]) -> bool:
    gid = nav.get("goal_id")
    if gid is None:
        return True
    return not str(gid).strip()


def navigate_failure_kind(nav: Mapping[str, Any]) -> Optional[str]:
    """
    Return ``NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE`` iff ``nav`` matches the frozen
    V3.0.1 advisory blocked outcome (full dict) or the bounded bridge-normalized equivalent;
    else ``None``.
    """
    if not isinstance(nav, dict):
        return None
    if (
        str(nav.get("outcome", "")).strip() == BLOCKED_OUTCOME_VALUE
        and str(nav.get("reason_code", "")).strip() == BLOCKED_REASON_CODE
    ):
        return NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE
    # Do not infer from message if semantic fields are partially present but did not match.
    if str(nav.get("outcome", "")).strip() or str(nav.get("reason_code", "")).strip():
        return None
    st = str(nav.get("status", "")).strip().lower()
    if st not in ("failed", "failure"):
        return None
    if str(nav.get("message", "")).strip() != BLOCKED_PEER_BELIEF_FAILURE_MESSAGE:
        return None
    if not _goal_id_empty(nav):
        return None
    return NAVIGATE_FAILURE_KIND_ADVISORY_BLOCKED_PASSAGE
