#!/usr/bin/env bash
# Golden-path demo: Layer B checked inspect (offline) and optional checked execute (ROS).
# Usage:
#   ./scripts/demo_layer_b_golden_path.sh           # offline inspect paths only
#   ./scripts/demo_layer_b_golden_path.sh --with-ros  # all four paths (needs live stack)

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="${ROOT}/src:${PYTHONPATH:-}"

run_py() {
  local label="$1"
  shift
  echo "== ${label}"
  python3 "$@"
}

run_py "Single-spec inspect (checked)" scripts/validate_coordinator_inspect_checked.py
run_py "Batch inspect (checked)" scripts/validate_coordinator_batch_inspect_checked.py

if [[ "${1:-}" == "--with-ros" ]]; then
  run_py "Single-spec execute (checked)" scripts/validate_coordinator_execute_checked_ros.py
  run_py "Batch execute (checked)" scripts/validate_coordinator_execute_batch_checked_ros.py
else
  echo "== Skipping execute (checked) demos (pass --with-ros when stack is running)."
fi

echo "Golden path demo finished OK."
