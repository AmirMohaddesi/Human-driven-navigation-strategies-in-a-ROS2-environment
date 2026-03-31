#!/usr/bin/env bash
# Cleanup helper for locked golden-path baseline runs.
# Use before launch attempts and after validation teardown.

set -euo pipefail

MODE="${1:-cleanup}"
if [[ "${MODE}" != "cleanup" && "${MODE}" != "pre" && "${MODE}" != "post" && "${MODE}" != "verify" ]]; then
  echo "usage: $0 [cleanup|pre|post|verify]" >&2
  exit 2
fi

PATTERN="fully_integrated_swarm.launch.py|gzserver|gzclient|rviz2|entity_spawner|lifecycle_manager|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|map_server|amcl|slam_toolbox|robot_state_publisher|initial_pose_publisher|merge_map_node|mission_bridge_node"
collect_remaining() {
  ps -eo pid=,args= | awk -v pat="$PATTERN" -v self="$$" -v parent="$PPID" '
    $0 ~ pat {
      pid = $1
      if (pid != self && pid != parent &&
          $0 !~ /cleanup_baseline_runtime\.sh/ &&
          $0 !~ /awk -v pat=/) {
        print
      }
    }
  '
}

if [[ "${MODE}" != "verify" ]]; then
  echo "[cleanup:${MODE}] stopping prior integrated launch and common child processes..."
  pkill -f "ros2 launch multi_robot_mission_stack fully_integrated_swarm.launch.py" || true
  pkill -f "$PATTERN" || true
  sleep 1

  remaining="$(collect_remaining)"
  if [[ -n "${remaining}" ]]; then
    echo "[cleanup:${MODE}] escalating: force-killing remaining matching processes..."
    pkill -9 -f "$PATTERN" || true
    sleep 1
  fi
fi

remaining_after="$(collect_remaining)"
if [[ -n "${remaining_after}" ]]; then
  echo "[cleanup:${MODE}] remaining matching processes after cleanup (NOT EMPTY):"
  printf '%s\n' "${remaining_after}"
  exit 1
fi

echo "[cleanup:${MODE}] remaining matching processes after cleanup: none"
