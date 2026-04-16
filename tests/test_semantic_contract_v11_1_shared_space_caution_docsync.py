from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any


_UUID4_RE = re.compile(
    r"^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$"
)
_RFC3339_UTC_Z_RE = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$")


def _load_fixture() -> dict[str, Any]:
    p = (
        Path(__file__).resolve().parent
        / "fixtures"
        / "semantic_v11_1_shared_space_caution_contract_examples.json"
    )
    return json.loads(p.read_text(encoding="utf-8"))


def _validate_record(
    rec: dict[str, Any], *, allowed_caution: set[str], expected_fact_type: str
) -> bool:
    required_keys = {
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
        "caution_class",
    }
    if set(rec.keys()) != required_keys:
        return False
    if rec["schema_version"] != "v11.1":
        return False
    if rec["fact_type"] != expected_fact_type:
        return False
    if not isinstance(rec["source_robot_id"], str) or not rec["source_robot_id"].strip():
        return False
    if not isinstance(rec["location_ref"], str) or not rec["location_ref"].strip():
        return False
    if rec["verification_status"] != "unverified":
        return False
    if not _UUID4_RE.fullmatch(str(rec["belief_id"])):
        return False
    if not _RFC3339_UTC_Z_RE.fullmatch(str(rec["timestamp_utc"])):
        return False

    conf = rec["confidence"]
    if isinstance(conf, bool) or not isinstance(conf, (int, float)):
        return False
    if float(conf) < 0.0 or float(conf) > 1.0:
        return False

    ttl = rec["ttl_sec"]
    if isinstance(ttl, bool) or not isinstance(ttl, (int, float)):
        return False
    if float(ttl) <= 0.0:
        return False

    cclass = rec["caution_class"]
    if not isinstance(cclass, str) or cclass not in allowed_caution:
        return False

    prov = rec["provenance"]
    if not isinstance(prov, dict):
        return False
    if set(prov.keys()) != {"sensor_class", "observation_id"}:
        return False
    if not isinstance(prov["sensor_class"], str) or not prov["sensor_class"].strip():
        return False
    if not _UUID4_RE.fullmatch(str(prov["observation_id"])):
        return False

    return True


def test_v111_fixture_header_is_frozen() -> None:
    fixture = _load_fixture()
    assert fixture["schema_version"] == "v11.1.fixture.1"
    assert fixture["fact_schema_version"] == "v11.1"
    assert fixture["fact_type"] == "shared_space_caution"


def test_v111_fixture_cases_match_expectations() -> None:
    fixture = _load_fixture()
    allowed = set(fixture["allowed_caution_class"])
    fact_type = str(fixture["fact_type"])

    for case in fixture["cases"]:
        expect = str(case["expect"])
        record = case["record"]
        valid = _validate_record(
            record,
            allowed_caution=allowed,
            expected_fact_type=fact_type,
        )
        if expect == "accept":
            assert valid, f"expected accept for case_id={case['case_id']}"
        elif expect == "reject":
            assert not valid, f"expected reject for case_id={case['case_id']}"
        else:
            raise AssertionError(f"unsupported expect value for case_id={case['case_id']}")
