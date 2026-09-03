#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

mkdir -p logs

WANT_VIEWER="${SPACEMOUSE_TELEOP_VIEWER:-1}"

if [[ -n "${PYTHON_BIN:-}" ]]; then
  PYTHON="$PYTHON_BIN"
elif [[ "$WANT_VIEWER" != "0" && -x ".venv/bin/mjpython" ]]; then
  PYTHON=".venv/bin/mjpython"
elif [[ -x ".venv/bin/python" ]]; then
  PYTHON=".venv/bin/python"
elif [[ "$WANT_VIEWER" != "0" ]] && command -v mjpython >/dev/null 2>&1; then
  PYTHON="$(command -v mjpython)"
else
  PYTHON="python"
fi

export PYTHONPATH="$ROOT_DIR/src${PYTHONPATH:+:$PYTHONPATH}"

ARGS=(
  scripts/mujoco_teleop.py
  --backend pyspacemouse
  --axis-convention ros
  --hz 60
  --config configs/spacemouse_xarm6_mujoco.json
  --target-mode velocity
  --arm-control-mode kinematic
  --position-gain 0.6
  --orientation-gain 0.35
  --print-rate 5
  --log logs/mujoco_spacemouse.jsonl
)

if [[ "$WANT_VIEWER" != "0" ]]; then
  ARGS+=(--viewer --camera rear_side --multiview)
fi

exec "$PYTHON" "${ARGS[@]}" "$@"
