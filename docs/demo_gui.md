# Demo GUI (Streamlit)

This is a very small Streamlit GUI wrapper around `scripts/demo_step_by_step_mission.py`.

It is intentionally minimal: choose a mode, click run, and view status + JSON output.

## How to run

From repo root:

```bash
pip install streamlit
streamlit run scripts/demo_gui.py
```

## What it does

- Shows a mode dropdown:
  - Single (dry-run)
  - Single (live)
  - Batch (dry-run)
  - Batch (live)
- Optional checkbox:
  - Pause between steps
- Runs the existing demo script using `subprocess.run(...)`
- Displays:
  - success/failure status
  - parsed JSON output with `st.json` (when available)

## Notes

- Live modes require the ROS mission stack to already be running.
- This is a minimal GUI wrapper for demos, not a production UI.

