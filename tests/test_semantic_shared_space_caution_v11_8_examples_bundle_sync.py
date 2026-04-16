"""V11.8 — checked-in canonical JSON examples match existing local public APIs (no ROS)."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

from multi_robot_mission_stack.semantic.shared_space_caution_candidate_v115 import (
    SharedSpaceCautionAssemblyCandidate,
    assemble_shared_space_caution_record_from_candidate,
)
from multi_robot_mission_stack.semantic.shared_space_caution_v111 import SharedSpaceCautionBeliefStore

_ROOT = Path(__file__).resolve().parents[1]
_BUNDLE = _ROOT / "docs" / "architecture" / "examples" / "v11_8_shared_space_caution_local"

_NOW = datetime(2026, 4, 8, 12, 30, 30, tzinfo=timezone.utc)
_BID = "123e4567-e89b-42d3-a456-426614174000"
_OID = "223e4567-e89b-42d3-a456-426614174111"


def test_v11_8_candidate_json_matches_assembly_input_shape() -> None:
    raw = json.loads((_BUNDLE / "candidate.canonical.json").read_text(encoding="utf-8"))
    cand = SharedSpaceCautionAssemblyCandidate(
        location_ref=raw["location_ref"],
        source_robot_id=raw["source_robot_id"],
        confidence=raw["confidence"],
        ttl_sec=raw["ttl_sec"],
        caution_class=raw["caution_class"],
        sensor_class=raw["sensor_class"],
    )
    out = assemble_shared_space_caution_record_from_candidate(
        cand, timestamp_utc=_NOW, belief_id=_BID, observation_id=_OID
    )
    assert out.accepted
    expected = json.loads((_BUNDLE / "assembled_record.canonical.json").read_text(encoding="utf-8"))
    assert out.record == expected


def test_v11_8_advisory_report_json_matches_store_query() -> None:
    record = json.loads((_BUNDLE / "assembled_record.canonical.json").read_text(encoding="utf-8"))
    store = SharedSpaceCautionBeliefStore(allowed_source_robot_ids=frozenset({"robot1"}))
    assert store.ingest(record, now_utc=_NOW).stored
    rep = store.query_advisory_shared_space_caution("base", now_utc=_NOW)
    expected = json.loads((_BUNDLE / "advisory_report.canonical.json").read_text(encoding="utf-8"))
    assert rep.to_dict() == expected
