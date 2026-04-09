"""V8.2 deterministic local validator scaffold tests for degraded_passage."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from multi_robot_mission_stack.semantic.degraded_passage_v81 import (
    FACT_TYPE_DEGRADED_PASSAGE,
    SCHEMA_VERSION,
    validate_degraded_passage_record,
)


def _load_v81_examples() -> dict[str, Any]:
    p = (
        Path(__file__).resolve().parent
        / "fixtures"
        / "semantic_v8_1_degraded_passage_contract_examples.json"
    )
    return json.loads(p.read_text(encoding="utf-8"))


def test_v82_validator_matches_v81_fixture_examples() -> None:
    data = _load_v81_examples()
    assert data["fact_schema_version"] == SCHEMA_VERSION
    assert data["fact_type"] == FACT_TYPE_DEGRADED_PASSAGE

    for case in data["cases"]:
        ok, errors = validate_degraded_passage_record(case["record"])
        expect = case["expect"]
        if expect == "accept":
            assert ok, f"case_id={case['case_id']} expected accept, got errors={errors}"
        elif expect == "reject":
            assert not ok, f"case_id={case['case_id']} expected reject"
            assert errors, f"case_id={case['case_id']} expected non-empty errors"
        else:
            raise AssertionError(f"unsupported expect value: {expect!r}")


def test_v82_validator_rejects_unknown_top_level_field() -> None:
    data = _load_v81_examples()
    rec = dict(data["cases"][0]["record"])
    rec["unexpected"] = "nope"
    ok, errors = validate_degraded_passage_record(rec)
    assert not ok
    assert any("unknown fields" in e for e in errors)
