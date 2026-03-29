#!/usr/bin/env bash
# Minimal cancel-flow verification; requires sourced ROS + workspace + live graph.
set -eo pipefail
source /opt/ros/humble/setup.bash
[[ -f "$(dirname "$0")/../install/setup.bash" ]] && source "$(dirname "$0")/../install/setup.bash"

log() { echo "[$(date +%H:%M:%S)] $*"; }

extract_gid() {
  grep '^{' | tail -1 | python3 -c "import json,sys; print(json.load(sys.stdin)['goal_id'])"
}

log "Step 1: direct cancel unknown goal_id"
SECONDS=0
set +e
out1=$(ros2 service call /cancel_navigation multi_robot_mission_stack_interfaces/srv/CancelNavigation "{robot_id: 'robot1', goal_id: '00000000-0000-0000-0000-000000000000'}" 2>&1)
ec1=$?
set -e
t1=$SECONDS
log "exit=$ec1 time=${t1}s"
printf '%s\n' "$out1"

log "Step 2: CLI navigate (goal_id GID)"
SECONDS=0
set +e
nav1=$(ros2 run multi_robot_mission_stack mission-agent --ros navigate --robot-id robot1 --location-name base 2>&1)
ec2=$?
set -e
t2=$SECONDS
log "exit=$ec2 time=${t2}s"
printf '%s\n' "$nav1"
GID=$(printf '%s\n' "$nav1" | extract_gid)

log "Step 3: direct cancel with GID=$GID"
SECONDS=0
set +e
out3=$(ros2 service call /cancel_navigation multi_robot_mission_stack_interfaces/srv/CancelNavigation "{robot_id: 'robot1', goal_id: '$GID'}" 2>&1)
ec3=$?
set -e
t3=$SECONDS
log "exit=$ec3 time=${t3}s"
printf '%s\n' "$out3"

log "Step 4: CLI navigate again (GID2)"
SECONDS=0
set +e
nav2=$(ros2 run multi_robot_mission_stack mission-agent --ros navigate --robot-id robot1 --location-name base 2>&1)
ec4=$?
set -e
t4=$SECONDS
log "exit=$ec4 time=${t4}s"
printf '%s\n' "$nav2"
GID2=$(printf '%s\n' "$nav2" | extract_gid)

log "Step 5: CLI cancel GID2=$GID2"
SECONDS=0
set +e
can=$(ros2 run multi_robot_mission_stack mission-agent --ros cancel --robot-id robot1 --goal-id "$GID2" 2>&1)
ec5=$?
set -e
t5=$SECONDS
log "exit=$ec5 time=${t5}s"
printf '%s\n' "$can"

log "Step 6: CLI query-state GID2"
SECONDS=0
set +e
qs=$(ros2 run multi_robot_mission_stack mission-agent --ros query-state --robot-id robot1 --goal-id "$GID2" 2>&1)
ec6=$?
set -e
t6=$SECONDS
log "exit=$ec6 time=${t6}s"
printf '%s\n' "$qs"

echo "=== SUMMARY exit_codes: step1=$ec1 step2=$ec2 step3=$ec3 step4=$ec4 step5=$ec5 step6=$ec6 ==="
echo "=== TIMES_SEC: t1=$t1 t2=$t2 t3=$t3 t4=$t4 t5=$t5 t6=$t6 ==="
