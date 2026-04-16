"""P3.3 — shell wrapper syntax and dry-run output (no ROS required)."""

from __future__ import annotations

import subprocess
from pathlib import Path


def _script() -> Path:
    return Path(__file__).resolve().parents[1] / "scripts" / "p3_3_operator_p3_2_visibility_demo.sh"


def test_p33_wrapper_passes_bash_n() -> None:
    p = _script()
    r = subprocess.run(["bash", "-n", str(p)], capture_output=True, text=True, check=False)
    assert r.returncode == 0, r.stderr


def test_p33_wrapper_dry_run_exits_zero_and_prints_sequence() -> None:
    p = _script()
    r = subprocess.run(
        ["bash", str(p), "--dry-run"],
        cwd=str(p.parent.parent),
        capture_output=True,
        text=True,
        check=False,
    )
    assert r.returncode == 0, r.stderr + r.stdout
    out = r.stdout
    assert "P3.3 — P3.2 visibility-only demo wrapper" in out
    assert "ros2 launch multi_robot_mission_stack p3_2_dual_advisory_visibility.launch.py" in out
    assert "+ python3 scripts/p1_1_print_blocked_advisory_wire.py" in out
    assert "+ python3 scripts/p1_1_print_degraded_advisory_wire.py" in out
    assert "+ ros2 topic pub -1 /semantic/blocked_passage_p1_1 std_msgs/msg/String" in out
