#!/usr/bin/env bash
# verify_navigate_named: mock + ROS named-location navigate, then query-state.
# NOTE: This script only exercises the current golden path:
#   - navigate to named location
#   - query navigation state
# NOTE: even if additional CLI subcommands exist, this script intentionally
# validates only the current golden path.
set -eo pipefail
cd "$(dirname "$0")/.."
export PYTHONPATH="${PWD}/src/multi_robot_mission_stack"

extract_gid() {
  grep '^{' | tail -1 | python3 -c "import json,sys; print(json.load(sys.stdin)['goal_id'])"
}

echo "=== 1 mock navigate(named-location) ==="
set +e
out1=$(python3 -m multi_robot_mission_stack.agent.cli --mock navigate --robot-id robot1 --location-name base 2>&1)
e1=$?
set -e
echo "exit=$e1"
echo "$out1"

echo "=== 2 mock query-state (using mock goal_id) ==="
MOCK_GID=$(printf '%s\n' "$out1" | extract_gid || true)
set +e
out2=$(python3 -m multi_robot_mission_stack.agent.cli --mock query-state --robot-id robot1 --goal-id "${MOCK_GID:-mock-goal-001}" 2>&1)
e2=$?
set -e
echo "exit=$e2"
echo "$out2"

source /opt/ros/humble/setup.bash 2>/dev/null || true
[[ -f "${PWD}/install/setup.bash" ]] && source "${PWD}/install/setup.bash"

echo "=== 3 ROS navigate(named-location) ==="
set +e
out3=$(ros2 run multi_robot_mission_stack mission-agent --ros navigate --robot-id robot1 --location-name base 2>&1)
e3=$?
set -e
echo "exit=$e3"
echo "$out3"
GID=$(printf '%s\n' "$out3" | extract_gid || true)

echo "=== 4 ROS query-state GID=$GID ==="
set +e
out4=$(ros2 run multi_robot_mission_stack mission-agent --ros query-state --robot-id robot1 --goal-id "$GID" 2>&1)
e4=$?
set -e
echo "exit=$e4"
echo "$out4"

echo "=== SUMMARY e1=$e1 e2=$e2 e3=$e3 e4=$e4 ==="
