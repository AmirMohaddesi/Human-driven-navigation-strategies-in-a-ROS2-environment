#!/usr/bin/env bash
# Thin wrapper: CLI wrong-robot cancel validator (see validate_mission_cli_cancel_wrong_robot_ros.py).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec python3 "$ROOT/scripts/validate_mission_cli_cancel_wrong_robot_ros.py" "$@"
