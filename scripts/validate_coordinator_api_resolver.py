#!/usr/bin/env python3
"""
Smoke-test resolve_team_named_mission_api_entrypoint (pure Python, no ROS).

  python3 scripts/validate_coordinator_api_resolver.py
"""

from __future__ import annotations

import json
import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from multi_robot_mission_stack.coordinator import resolve_team_named_mission_api_entrypoint  # noqa: E402


def main() -> int:
    blob = resolve_team_named_mission_api_entrypoint("single_spec", "execute_checked")
    print(json.dumps(blob, separators=(",", ":")))
    return 0 if blob.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
