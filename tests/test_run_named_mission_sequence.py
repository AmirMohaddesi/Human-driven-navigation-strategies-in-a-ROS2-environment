"""Unit tests for scripts/run_named_mission_sequence.py step parsing."""

from __future__ import annotations

import importlib.util
from pathlib import Path

_MOD = Path(__file__).resolve().parents[1] / "scripts" / "run_named_mission_sequence.py"
_spec = importlib.util.spec_from_file_location("run_named_mission_sequence", _MOD)
assert _spec and _spec.loader
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)


def test_parse_step_ok() -> None:
    assert _mod.parse_step("robot1:base") == ("robot1", "base")
    assert _mod.parse_step(" robot2 : test_goal ") == ("robot2", "test_goal")


def test_parse_step_rejects_invalid() -> None:
    import pytest

    with pytest.raises(ValueError):
        _mod.parse_step("robot1")
    with pytest.raises(ValueError):
        _mod.parse_step(":base")
    with pytest.raises(ValueError):
        _mod.parse_step("robot1:")


def test_parse_last_json_line() -> None:
    assert _mod.parse_last_json_line('noise\n{"a":1}\n') == {"a": 1}
    assert _mod.parse_last_json_line("") is None
    assert _mod.parse_last_json_line("not json") is None


def test_step_succeeded() -> None:
    ok, d = _mod.step_succeeded(
        0,
        '{"outcome":"succeeded","polls":1}\n',
    )
    assert ok and d and d["outcome"] == "succeeded"
    ok2, _ = _mod.step_succeeded(0, '{"outcome":"cancelled"}')
    assert not ok2
    ok3, det3 = _mod.step_succeeded(1, '{"outcome":"succeeded"}')
    assert not ok3
    assert det3 == {"outcome": "succeeded"}
