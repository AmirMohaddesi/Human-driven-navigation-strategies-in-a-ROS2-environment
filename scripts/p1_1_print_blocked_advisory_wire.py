#!/usr/bin/env python3
"""Print one line of compact JSON for a valid V3.0.1 ``blocked_passage`` advisory wire payload."""

from __future__ import annotations

import time
import uuid

from multi_robot_mission_stack.semantic.blocked_passage_v301 import SCHEMA_VERSION
from multi_robot_mission_stack.transport.blocked_passage_json_v301 import encode_blocked_passage_record_json


def _to_iso_z(now: float) -> str:
    from datetime import datetime, timezone

    dt = datetime.fromtimestamp(now, tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S") + ".%03dZ" % (dt.microsecond // 1000)


def main() -> None:
    now = time.time()
    record = {
        "schema_version": SCHEMA_VERSION,
        "belief_id": str(uuid.uuid4()),
        "fact_type": "blocked_passage",
        "source_robot_id": "robot1",
        "timestamp_utc": _to_iso_z(now),
        "confidence": 0.85,
        "location_ref": "base",
        "provenance": {
            "sensor_class": "lidar_occlusion",
            "observation_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
        },
        "ttl_sec": 3600.0,
        "verification_status": "unverified",
    }
    print(encode_blocked_passage_record_json(record), flush=True)


if __name__ == "__main__":
    main()
