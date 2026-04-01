#!/usr/bin/env bash
# Thin wrapper: CLI invalid-goal cancel validator (see validate_mission_cli_cancel_invalid_goal_ros.py).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec python3 "$ROOT/scripts/validate_mission_cli_cancel_invalid_goal_ros.py" "$@"
