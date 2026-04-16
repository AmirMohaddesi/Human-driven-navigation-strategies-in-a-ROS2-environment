#!/usr/bin/env bash
# P3.3 — operator-facing wrapper for the P3.2 visibility-only dual-advisory board demo.
#
# Usage:
#   Terminal 1: source install/setup.bash && ros2 launch multi_robot_mission_stack p3_2_dual_advisory_visibility.launch.py
#   Terminal 2: cd <repo> && source install/setup.bash && ./scripts/p3_3_operator_p3_2_visibility_demo.sh
#
#   ./scripts/p3_3_operator_p3_2_visibility_demo.sh --dry-run   # print commands only, no ros2/python side effects

set -eu

DRY_RUN=0
if [ "${1:-}" = "--dry-run" ]; then
  DRY_RUN=1
  shift
fi
if [ "${1:-}" != "" ]; then
  echo "usage: $0 [--dry-run]" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

TOPIC_BLOCKED="/semantic/blocked_passage_p1_1"
TOPIC_DEGRADED="/semantic/degraded_passage_p1_1"
MALFORMED_PAYLOAD="{data: '{not-json'}"

run() {
  printf '+ %s\n' "$*"
  if [ "$DRY_RUN" -eq 0 ]; then
    eval "$1"
  fi
}

section() {
  printf '\n=== %s ===\n' "$1"
}

echo "================================================================"
echo " P3.3 — P3.2 visibility-only demo wrapper (blocked/degraded/malformed)"
echo "================================================================"
echo
echo "HONESTY (read this):"
echo "  - Visibility-only wrapper around existing P3.2 + P1.1 helper scripts."
echo "  - No mission goals, no cancel, no service/action calls, no control authority."
echo "  - No new semantic meaning; publishes only to existing P1.1 advisory topics."
echo "  - Not a dashboard or mission operator tool."
echo
if [ "$DRY_RUN" -eq 1 ]; then
  echo "(dry-run mode: commands are printed with a leading '+'; nothing is executed.)"
  echo
fi

section "Terminal 1 — start P3.2 launch (leave it running)"
echo "  source install/setup.bash"
echo "  ros2 launch multi_robot_mission_stack p3_2_dual_advisory_visibility.launch.py"
echo

if [ "$DRY_RUN" -eq 0 ]; then
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "error: 'ros2' not on PATH. Source install/setup.bash first." >&2
    exit 1
  fi
  read -r -p "Press Enter when P3.2 launch is running..." _
fi

section "Step A — publish valid blocked advisory"
cmd_a="python3 scripts/p1_1_print_blocked_advisory_wire.py | python3 scripts/p1_1_publish_std_string_once.py ${TOPIC_BLOCKED}"
run "$cmd_a"

section "Expected on P3.2 board after Step A"
echo "  - blocked row updates with status VALID and fact_type blocked_passage"
echo "  - blocked valid counter increments"
echo

section "Step B — publish valid degraded advisory"
cmd_b="python3 scripts/p1_1_print_degraded_advisory_wire.py | python3 scripts/p1_1_publish_std_string_once.py ${TOPIC_DEGRADED}"
run "$cmd_b"

section "Expected on P3.2 board after Step B"
echo "  - degraded row updates with status VALID and fact_type degraded_passage"
echo "  - degraded valid counter increments"
echo

section "Step C — publish malformed advisory payload (blocked topic)"
cmd_c="ros2 topic pub -1 ${TOPIC_BLOCKED} std_msgs/msg/String \"${MALFORMED_PAYLOAD}\""
run "$cmd_c"

section "Expected on P3.2 board after Step C"
echo "  - blocked malformed counter increments"
echo "  - recent events include status UNPARSEABLE_JSON"
echo "  - board remains alive (no crash)"
echo

echo "Done. Wrapper checkpoint: docs/architecture/p3_3_p3_2_visibility_demo_wrapper_checkpoint.md"
