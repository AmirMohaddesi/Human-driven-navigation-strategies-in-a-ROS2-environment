"""P3.4 — static cheatsheet anchor coverage (no ROS required)."""

from __future__ import annotations

from pathlib import Path


def _cheatsheet() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "docs"
        / "architecture"
        / "p3_4_dual_advisory_visibility_interpretation_cheatsheet.md"
    )


def test_p34_cheatsheet_exists_and_contains_required_markers() -> None:
    p = _cheatsheet()
    assert p.exists(), "P3.4 cheatsheet is missing"
    text = p.read_text(encoding="utf-8")
    assert "[P3.1]" in text
    assert "[P3.2_ROW]" in text
    assert "[P3.2_EVT]" in text
    assert "blocked -> degraded -> malformed" in text
