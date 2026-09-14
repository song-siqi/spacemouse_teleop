#!/usr/bin/env bash
set -euo pipefail

MOVE_POSE_REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -x "$MOVE_POSE_REPO_DIR/.venv/bin/python" ]]; then
  echo "Missing project .venv; install the existing xarm dependency first." >&2
  exit 1
fi
export PYTHONPATH="$MOVE_POSE_REPO_DIR/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$MOVE_POSE_REPO_DIR/.venv/bin/python" -u -m spacemouse_teleop.cli.xarm_move_to_pose "$@"
