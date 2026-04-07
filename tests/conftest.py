"""Shared pytest configuration: make ``multi_robot_mission_stack`` importable without install."""

from __future__ import annotations

import os
import sys

# ROS package root: parent of the inner ``multi_robot_mission_stack`` Python package.
_PKG_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "src", "multi_robot_mission_stack")
)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)
