#!/usr/bin/env python3
"""
Validate mission contract v1 helpers and API manifest (pure Python, no ROS).

  python3 scripts/validate_mission_contract_v1.py
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
    validate_team_named_mission_spec_contract,
    validate_team_named_mission_specs_contract,
)


def main() -> int:
    single = {
        "version": "v1",
        "mode": "sequence",
        "steps": [{"robot_id": "r1", "location_name": "base"}],
    }
    batch = [
        single,
        {
            "version": "v1",
            "mode": "parallel",
            "steps": [
                {"robot_id": "r1", "location_name": "a"},
                {"robot_id": "r2", "location_name": "b"},
            ],
        },
    ]

    manifest = get_team_named_mission_api_manifest()
    out = {
        "single_spec_contract": validate_team_named_mission_spec_contract(single),
        "batch_specs_contract": validate_team_named_mission_specs_contract(batch),
        "manifest": manifest,
        "manifest_validation": validate_team_named_mission_api_manifest(manifest),
    }

    print(json.dumps(out, separators=(",", ":"), default=str))

    sc = out["single_spec_contract"]
    bc = out["batch_specs_contract"]
    mv = out["manifest_validation"]
    s_ok = isinstance(sc, dict) and sc.get("ok") is True
    b_ok = isinstance(bc, dict) and bc.get("ok") is True
    m_ok = isinstance(mv, dict) and mv.get("ok") is True

    return 0 if s_ok and b_ok and m_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
