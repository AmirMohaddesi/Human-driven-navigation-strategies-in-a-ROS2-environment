#!/usr/bin/env bash
# Thin wrapper: CLI ownership-safe cancel validator (see validate_mission_cli_ownership_cancel_ros.py).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec python3 "$ROOT/scripts/validate_mission_cli_ownership_cancel_ros.py" "$@"
