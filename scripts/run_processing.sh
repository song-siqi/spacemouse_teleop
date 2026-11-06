#!/usr/bin/env bash
set -euo pipefail
PROCESS_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$PROCESS_REPO/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$PROCESS_REPO/.venv-processing/bin/python" -m robot_teleop.cli.main "$@"
