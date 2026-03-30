# Golden-path demo: checked inspect / execute (single + batch)

**Purpose:** Repeatable demo of the four “checked” coordinator flows.  
**Package:** `multi_robot_mission_stack.coordinator`

## Prerequisites

| Flow | Requires |
|------|----------|
| Single inspect checked | Python + package on `PYTHONPATH` only |
| Batch inspect checked | Same |
| Single execute checked | Sourced workspace + running mission bridge / Nav2 as used elsewhere |
| Batch execute checked | Same |

## One-shot script

From the repository root:

```bash
# Offline-only (CI / laptop without ROS): inspect checked ×2
./scripts/demo_layer_b_golden_path.sh

# Full four-step demo (fails if ROS stack not ready)
./scripts/demo_layer_b_golden_path.sh --with-ros
```

## Manual equivalents

```bash
cd /path/to/HDNS   # repository root
export PYTHONPATH="/path/to/HDNS/src:${PYTHONPATH}"

# 1) Single-spec inspect (checked)
python3 scripts/validate_coordinator_inspect_checked.py

# 2) Single-spec execute (checked) — live
python3 scripts/validate_coordinator_execute_checked_ros.py

# 3) Batch inspect (checked)
python3 scripts/validate_coordinator_batch_inspect_checked.py

# 4) Batch execute (checked) — live
python3 scripts/validate_coordinator_execute_batch_checked_ros.py
```

**Success:** each script prints compact JSON and exits `0` only when top-level `ok` is true.

## What “checked” means (demo level)

Each checked helper returns the primary coordinator result **plus** a separate validation payload so callers can assert invariants without re-deriving them.
