#!/usr/bin/env bash
# verify_navigate_pose: mock + ROS navigate-pose, query-state, cancel; optional invalid path
set -eo pipefail
cd "$(dirname "$0")/.."
export PYTHONPATH="${PWD}/src/multi_robot_mission_stack"

extract_gid() {
  grep '^{' | tail -1 | python3 -c "import json,sys; print(json.load(sys.stdin)['goal_id'])"
}

echo "=== 1 mock navigate-pose ==="
set +e
out1=$(python3 -m multi_robot_mission_stack.agent.cli --mock navigate-pose --robot-id robot1 --x 1.0 --y 2.0 --yaw 0.0 2>&1)
e1=$?
set -e
echo "exit=$e1"
echo "$out1"

source /opt/ros/humble/setup.bash 2>/dev/null || true
[[ -f "${PWD}/install/setup.bash" ]] && source "${PWD}/install/setup.bash"

echo "=== 2 ROS navigate-pose ==="
set +e
out2=$(ros2 run multi_robot_mission_stack mission-agent --ros navigate-pose --robot-id robot1 --x 0.5 --y 0.0 --yaw 0.0 2>&1)
e2=$?
set -e
echo "exit=$e2"
echo "$out2"
GID=$(printf '%s\n' "$out2" | extract_gid) || true

echo "=== 3 query-state GID=$GID ==="
set +e
out3=$(ros2 run multi_robot_mission_stack mission-agent --ros query-state --robot-id robot1 --goal-id "$GID" 2>&1)
e3=$?
set -e
echo "exit=$e3"
echo "$out3"

echo "=== 4 cancel GID ==="
set +e
out4=$(ros2 run multi_robot_mission_stack mission-agent --ros cancel --robot-id robot1 --goal-id "$GID" 2>&1)
e4=$?
set -e
echo "exit=$e4"
echo "$out4"

echo "=== 5 invalid: missing --x (argparse) ==="
set +e
out5=$(ros2 run multi_robot_mission_stack mission-agent --ros navigate-pose --robot-id robot1 --y 0.0 --yaw 0.0 2>&1)
e5=$?
set -e
echo "exit=$e5"
echo "$out5"

echo "=== SUMMARY e1=$e1 e2=$e2 e3=$e3 e4=$e4 e5=$e5 ==="
