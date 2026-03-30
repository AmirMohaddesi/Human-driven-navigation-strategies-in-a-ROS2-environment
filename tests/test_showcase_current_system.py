"""Minimal shape check for scripts/showcase_current_system.py output."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


def test_showcase_current_system_json_shape() -> None:
    root = Path(__file__).resolve().parents[1]
    script = root / "scripts" / "showcase_current_system.py"
    out = subprocess.check_output([sys.executable, str(script)], text=True, cwd=str(root))
    data = json.loads(out.strip())
    assert data["ok"] is True
    assert data["overall_outcome"] == "success"
    assert "layer_a" in data["layers"]
    assert "layer_b" in data["layers"]
    assert "layer_c" in data["layers"]
    assert "contracts" in data
    assert isinstance(data["demos"], list)
    assert isinstance(data["docs"], list)
    assert data["summary"]["message"]
    assert data["summary"]["error"] is None
