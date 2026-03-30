#!/usr/bin/env python3
"""
Tiny terminal launcher for step-by-step mission demos.

This is a thin wrapper over ``scripts/demo_step_by_step_mission.py`` only.
"""

from __future__ import annotations

import subprocess
import sys
from typing import List, Optional


def build_demo_launcher_command(choice: str) -> Optional[List[str]]:
    """
    Map a menu choice string to a demo_step_by_step_mission.py command.

    Returns a list suitable for subprocess.run, or None to indicate "Quit".
    """
    base = ["python3", "scripts/demo_step_by_step_mission.py"]
    c = choice.strip()

    if c == "1":
        # Single mission dry-run
        return base
    if c == "2":
        # Single mission live (ROS required)
        return base + ["--with-ros"]
    if c == "3":
        # Batch mission dry-run
        return base + ["--batch"]
    if c == "4":
        # Batch mission live (ROS required)
        return base + ["--batch", "--with-ros"]
    if c == "5":
        # Single mission dry-run with pause
        return base + ["--pause"]
    if c == "6":
        # Single mission live with pause
        return base + ["--with-ros", "--pause"]
    if c == "7":
        # Batch mission dry-run with pause
        return base + ["--batch", "--pause"]
    if c == "8":
        # Batch mission live with pause
        return base + ["--batch", "--with-ros", "--pause"]
    if c == "9":
        # Quit
        return None

    return []  # sentinel for "invalid choice"


def main() -> int:
    print("Step-by-step mission demo launcher")
    print("")
    print("1. Single mission dry-run")
    print("2. Single mission live (ROS required)")
    print("3. Batch mission dry-run")
    print("4. Batch mission live (ROS required)")
    print("5. Single mission dry-run with pause")
    print("6. Single mission live with pause")
    print("7. Batch mission dry-run with pause")
    print("8. Batch mission live with pause")
    print("9. Quit")
    print("")

    try:
        choice = input("Select an option (1-9): ")
    except EOFError:
        print("No input received.")
        return 1

    cmd = build_demo_launcher_command(choice)
    if cmd is None:
        return 0
    if cmd == []:
        print("Invalid choice. Exiting.")
        return 1

    try:
        result = subprocess.run(cmd)
    except FileNotFoundError as exc:
        print(f"Failed to launch demo: {exc}")
        return 1

    return int(result.returncode)


if __name__ == "__main__":
    raise SystemExit(main())

