#!/usr/bin/env python3
"""
Smoke-test get_team_named_mission_api_manifest + validate_team_named_mission_api_manifest (no ROS).

  python3 scripts/validate_coordinator_api_manifest_invariants.py
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.coordinator import (  # noqa: E402
    get_team_named_mission_api_manifest,
    validate_team_named_mission_api_manifest,
)


def main() -> int:
    manifest = get_team_named_mission_api_manifest()
    blob = validate_team_named_mission_api_manifest(manifest)
    print(json.dumps(blob, separators=(",", ":")))
    return 0 if blob.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
