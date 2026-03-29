#!/usr/bin/env bash
# Runtime checks for run_named_mission_sequence.py (source ROS + workspace first).
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
[[ -f install/setup.bash ]] && source install/setup.bash

echo ">>> seq A: robot1:base robot2:test_goal"
set +e
python3 scripts/run_named_mission_sequence.py \
  --steps robot1:base robot2:test_goal \
  --per-goal-timeout 120 \
  --poll-interval 1.0
echo "exit=$?"
set -e

echo ">>> seq B: bad location + continue-on-failure"
set +e
python3 scripts/run_named_mission_sequence.py \
  --steps robot1:base robot2:bad_location_xyz \
  --per-goal-timeout 30 \
  --poll-interval 0.5 \
  --continue-on-failure
echo "exit=$?"
set -e

echo ">>> seq C: robot1:base robot2:test_goal robot1:base"
set +e
python3 scripts/run_named_mission_sequence.py \
  --steps robot1:base robot2:test_goal robot1:base \
  --per-goal-timeout 120 \
  --poll-interval 1.0
echo "exit=$?"
set -e
