#!/usr/bin/env bash
set -euo pipefail

TRANSLATION_REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$TRANSLATION_REPO_DIR"
if [[ ! -x "$TRANSLATION_REPO_DIR/.venv/bin/python" ]]; then
  echo "Missing project .venv; install the existing spacemouse and xarm dependencies first." >&2
  exit 1
fi
export PYTHONPATH="$TRANSLATION_REPO_DIR/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$TRANSLATION_REPO_DIR/.venv/bin/python" -u -m robot_teleop.cli.xarm_translation "$@"
