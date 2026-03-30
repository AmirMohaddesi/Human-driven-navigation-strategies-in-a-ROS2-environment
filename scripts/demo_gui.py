#!/usr/bin/env python3
"""
Minimal Streamlit GUI for running the mission demo script.

Thin wrapper only; mission logic stays in demo_step_by_step_mission.py.
"""

from __future__ import annotations

import json
import subprocess
from typing import Any, Dict, List, Tuple

import streamlit as st


def build_command(mode: str, pause: bool) -> List[str]:
    cmd = ["python3", "scripts/demo_step_by_step_mission.py"]
    if mode == "Single (live)":
        cmd.append("--with-ros")
    elif mode == "Batch (dry-run)":
        cmd.append("--batch")
    elif mode == "Batch (live)":
        cmd.extend(["--batch", "--with-ros"])
    # "Single (dry-run)" is default command with no flags.
    if pause:
        cmd.append("--pause")
    return cmd


def parse_last_json(stdout_text: str) -> Tuple[Dict[str, Any] | None, str]:
    lines = [line.strip() for line in stdout_text.splitlines() if line.strip()]
    for line in reversed(lines):
        if line.startswith("{") and line.endswith("}"):
            try:
                payload = json.loads(line)
                if isinstance(payload, dict):
                    return payload, ""
            except json.JSONDecodeError:
                continue
    return None, "No JSON payload found in command output."


st.title("Mission System Demo (Layer C → B → A)")

mode = st.selectbox(
    "Mode",
    (
        "Single (dry-run)",
        "Single (live)",
        "Batch (dry-run)",
        "Batch (live)",
    ),
)

pause = st.checkbox("Pause between steps", value=False)

if st.button("Run Demo"):
    command = build_command(mode, pause)
    result = subprocess.run(command, capture_output=True, text=True)

    payload, parse_error = parse_last_json(result.stdout)
    ok = result.returncode == 0

    st.subheader("Result")
    if ok:
        st.success("Mission completed successfully")
    else:
        st.error("Mission failed")

    if payload is not None:
        st.subheader("Summary")
        st.write(f"overall_outcome: {payload.get('overall_outcome')}")
        if "mode" in payload:
            st.write(f"mode: {payload.get('mode')}")
        if "mission_state" in payload:
            st.write(f"mission_state: {payload.get('mission_state')}")
        summary = payload.get("summary")
        if isinstance(summary, dict) and "message" in summary:
            st.write(f"message: {summary.get('message')}")
        elif "message" in payload:
            st.write(f"message: {payload.get('message')}")

    if payload is not None:
        st.json(payload)
    else:
        st.write(parse_error)
        st.write("Raw stdout:")
        st.text(result.stdout)

    if result.stderr.strip():
        st.write("stderr:")
        st.text(result.stderr)

