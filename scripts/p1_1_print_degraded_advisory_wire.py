#!/usr/bin/env python3
"""Print one line of compact JSON for a valid V8.1 ``degraded_passage`` advisory wire payload."""

from __future__ import annotations

from datetime import datetime, timezone

from multi_robot_mission_stack.semantic.degraded_passage_candidate_v85 import (
    DegradedPassageAssemblyCandidate,
    assemble_degraded_passage_record_from_candidate,
)
from multi_robot_mission_stack.transport.degraded_passage_json_v87 import encode_degraded_passage_record_json


def main() -> None:
    now = datetime.now(timezone.utc)
    asm = assemble_degraded_passage_record_from_candidate(
        DegradedPassageAssemblyCandidate(
            location_ref="base",
            source_robot_id="robot1",
            confidence=0.7,
            ttl_sec=600.0,
            degradation_class="slow_zone",
            recommended_speed_factor=0.6,
            sensor_class="lidar_occlusion",
        ),
        timestamp_utc=now,
    )
    print(encode_degraded_passage_record_json(asm.record), flush=True)


if __name__ == "__main__":
    main()
