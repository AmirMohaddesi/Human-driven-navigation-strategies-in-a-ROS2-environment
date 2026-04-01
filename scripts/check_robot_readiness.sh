#!/usr/bin/env bash
# Per-robot Nav2 startup readiness for fully_integrated_swarm (operator helper).
# Does not run mission validation. Requires: source install/setup.bash (same shell).
#
# Usage: bash scripts/check_robot_readiness.sh [namespace]
# Default namespace: robot1_ns
#
# Exit codes: 0 = ready, 1 = not ready, 2 = partially ready, 3 = ros2/setup error

set -u

NS="${1:-robot1_ns}"
NS="${NS#/}"

usage() {
  echo "usage: $0 [namespace]" >&2
  echo "  default namespace: robot1_ns" >&2
  echo "  requires: ROS 2 env (e.g. source install/setup.bash)" >&2
}

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 not in PATH (source your workspace install/setup.bash)." >&2
  usage
  exit 3
fi

MAP_TOPIC="/${NS}/map"
TF_TOPIC="/${NS}/tf"
AMCL_NODE="/${NS}/amcl"

echo "=== Robot readiness: ${NS} ==="

lifecycle_line() {
  local short=$1
  local path="/${NS}/${short}"
  local out
  if ! out=$(ros2 lifecycle get "$path" 2>/dev/null); then
    echo "  ${short}: ERROR (no lifecycle or node missing)"
    return 1
  fi
  local first
  first=$(echo "$out" | head -1)
  if echo "$first" | grep -qE '^active \[3\]'; then
    echo "  ${short}: active"
    return 0
  fi
  echo "  ${short}: ${first}"
  return 1
}

ok_ms=0
ok_amcl=0
ok_bt=0
lifecycle_line map_server && ok_ms=1 || true
lifecycle_line amcl && ok_amcl=1 || true
lifecycle_line bt_navigator && ok_bt=1 || true

map_ok=0
if ros2 topic type "$MAP_TOPIC" >/dev/null 2>&1; then
  echo "  map topic ${MAP_TOPIC}: available"
  map_ok=1
else
  echo "  map topic ${MAP_TOPIC}: NOT available"
fi

line=$(ros2 topic info "$TF_TOPIC" 2>/dev/null | awk '/^Publisher count:/{print $3; exit}' || true)
pubs=${line:-0}
if [[ -z "${pubs}" || ! "$pubs" =~ ^[0-9]+$ ]]; then
  pubs=0
fi
if [[ "$pubs" -gt 0 ]]; then
  echo "  ${TF_TOPIC} publishers: ${pubs}"
else
  echo "  ${TF_TOPIC} publishers: 0 (none)"
fi

tf_broadcast_ok=0
tb_out=""
if tb_out=$(ros2 param get "$AMCL_NODE" tf_broadcast 2>/dev/null); then
  echo "  ${AMCL_NODE} tf_broadcast: ${tb_out}"
  if echo "$tb_out" | grep -q 'Boolean value is: False'; then
    tf_broadcast_ok=1
  fi
else
  echo "  ${AMCL_NODE} tf_broadcast: (param get failed)"
fi

slam_on_tf=0
if ros2 topic info "$TF_TOPIC" -v 2>/dev/null | grep -q 'Node name: slam_toolbox'; then
  slam_on_tf=1
  echo "  ${TF_TOPIC}: slam_toolbox listed as publisher (expected for map->odom here)"
else
  echo "  ${TF_TOPIC}: slam_toolbox not listed as publisher (unexpected for this stack)"
fi

lifecycle_ok=$((ok_ms * ok_amcl * ok_bt))
# not merged map — informational
echo "  note: /merged_map is not a per-robot Nav2 readiness gate (see runbook §J)"

echo "---"
conclusion="not ready"
exit_code=1

if [[ "$map_ok" -eq 0 || "$pubs" -eq 0 || "$lifecycle_ok" -eq 0 ]]; then
  conclusion="not ready"
  exit_code=1
elif [[ "$tf_broadcast_ok" -eq 1 && "$slam_on_tf" -eq 1 ]]; then
  conclusion="ready"
  exit_code=0
else
  conclusion="partially ready"
  exit_code=2
fi

echo "Conclusion: ${conclusion}"
exit "$exit_code"
