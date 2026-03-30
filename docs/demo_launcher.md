# Demo launcher (terminal menu)

This is a **tiny terminal wrapper**, not a GUI. It only launches the existing `scripts/demo_step_by_step_mission.py` script with different flag combinations so you do not have to remember them.

It does **not** change the Layer B contract, ROS behavior, or planner logic.

## How to run

From the repo root:

```bash
python3 scripts/demo_launcher.py
```

You will see a small numbered menu:

1. Single mission dry-run  
2. Single mission live (ROS required)  
3. Batch mission dry-run  
4. Batch mission live (ROS required)  
5. Single mission dry-run with pause  
6. Single mission live with pause  
7. Batch mission dry-run with pause  
8. Batch mission live with pause  
9. Quit  

Type a number (1–9) and press Enter.

## What each option does

- **1. Single mission dry-run**  
  Runs `python3 scripts/demo_step_by_step_mission.py`  
  (planner intent → v1 spec → checked inspect, no ROS execution).

- **2. Single mission live (ROS required)**  
  Runs `python3 scripts/demo_step_by_step_mission.py --with-ros`  
  (same as 1, then checked execute; requires ROS stack already running).

- **3. Batch mission dry-run**  
  Runs `python3 scripts/demo_step_by_step_mission.py --batch`  
  (tiny hardcoded batch → checked batch inspect, no ROS execution).

- **4. Batch mission live (ROS required)**  
  Runs `python3 scripts/demo_step_by_step_mission.py --batch --with-ros`  
  (checked batch inspect, then checked batch execute; ROS required).

- **5. Single mission dry-run with pause**  
  Runs `python3 scripts/demo_step_by_step_mission.py --pause`  
  (single dry-run, pausing after each phase).

- **6. Single mission live with pause**  
  Runs `python3 scripts/demo_step_by_step_mission.py --with-ros --pause`  
  (single live flow with pauses; ROS required).

- **7. Batch mission dry-run with pause**  
  Runs `python3 scripts/demo_step_by_step_mission.py --batch --pause`  
  (batch dry-run with pauses between phases).

- **8. Batch mission live with pause**  
  Runs `python3 scripts/demo_step_by_step_mission.py --batch --with-ros --pause`  
  (batch live flow with pauses; ROS required).

- **9. Quit**  
  Does not run any demo; the launcher exits with code 0.

## Notes

- Live options (2, 4, 6, 8) **require ROS/bridge/Nav2** to be running in the same way as the underlying demo script expects.
- This launcher is intentionally small and text-only; it is still just a **terminal wrapper**, not a TUI or graphical interface.

