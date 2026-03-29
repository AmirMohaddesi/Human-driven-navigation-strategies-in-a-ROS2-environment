#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
[[ -f install/setup.bash ]] && source install/setup.bash

extract_gid() {
  grep '^{' | tail -1 | python3 -c "import json,sys; print(json.load(sys.stdin)['goal_id'])"
}

echo "=== 1 pytest ==="
set +e
python3 -m pytest tests/test_wait_for_goal.py -q 2>&1
E1=$?
set -e
echo "exit=$E1"

echo "=== 2 unknown goal ==="
set +e
OUT2=$(python3 scripts/wait_for_goal.py --robot-id robot1 --goal-id 00000000-0000-0000-0000-000000000000 --timeout 10 --poll-interval 0.5 2>&1)
E2=$?
set -e
echo "exit=$E2"
echo "$OUT2"

echo "=== 3 real goal wait (terminal) ==="
set +e
N3=$(ros2 run multi_robot_mission_stack mission-agent --ros navigate --robot-id robot1 --location-name base 2>&1)
G3=$(printf '%s\n' "$N3" | extract_gid)
OUT3=$(python3 scripts/wait_for_goal.py --robot-id robot1 --goal-id "$G3" --timeout 120 --poll-interval 1.0 2>&1)
E3=$?
set -e
echo "exit=$E3 G3=$G3"
echo "$OUT3"

echo "=== 4 cancel then wait (cancelled) ==="
set +e
N4=$(ros2 run multi_robot_mission_stack mission-agent --ros navigate --robot-id robot1 --location-name base 2>&1)
G4=$(printf '%s\n' "$N4" | extract_gid)
ros2 run multi_robot_mission_stack mission-agent --ros cancel --robot-id robot1 --goal-id "$G4" >/dev/null 2>&1
OUT4=$(python3 scripts/wait_for_goal.py --robot-id robot1 --goal-id "$G4" --timeout 60 --poll-interval 0.5 2>&1)
E4=$?
set -e
echo "exit=$E4 G4=$G4"
echo "$OUT4"

echo "=== 5 aggressive timeout ==="
set +e
N5=$(ros2 run multi_robot_mission_stack mission-agent --ros navigate --robot-id robot1 --location-name base 2>&1)
G5=$(printf '%s\n' "$N5" | extract_gid)
OUT5=$(python3 scripts/wait_for_goal.py --robot-id robot1 --goal-id "$G5" --timeout 0.3 --poll-interval 0.1 2>&1)
E5=$?
set -e
echo "exit=$E5 G5=$G5"
echo "$OUT5"

echo "=== SUMMARY E1=$E1 E2=$E2 E3=$E3 E4=$E4 E5=$E5 ==="
