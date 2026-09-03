#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

mkdir -p logs

WANT_VIEWER="${SPACEMOUSE_TELEOP_VIEWER:-1}"
WANT_MULTIVIEW="${SPACEMOUSE_TELEOP_MULTIVIEW:-1}"
BACKEND="${SPACEMOUSE_TELEOP_BACKEND:-pyspacemouse}"
AXIS_CONVENTION="${SPACEMOUSE_TELEOP_AXIS_CONVENTION:-ros}"
HZ="${SPACEMOUSE_TELEOP_HZ:-60}"
CONFIG="${SPACEMOUSE_TELEOP_CONFIG:-configs/spacemouse_xarm6_mujoco.json}"
TARGET_MODE="${SPACEMOUSE_TELEOP_TARGET_MODE:-velocity}"
ARM_CONTROL_MODE="${SPACEMOUSE_TELEOP_ARM_CONTROL_MODE:-kinematic}"
POSITION_GAIN="${SPACEMOUSE_TELEOP_POSITION_GAIN:-0.6}"
ORIENTATION_GAIN="${SPACEMOUSE_TELEOP_ORIENTATION_GAIN:-0.35}"
PRINT_RATE="${SPACEMOUSE_TELEOP_PRINT_RATE:-5}"
LOG_PATH="${SPACEMOUSE_TELEOP_LOG:-logs/mujoco_spacemouse.jsonl}"
CAMERA="${SPACEMOUSE_TELEOP_CAMERA:-rear_side}"
MULTIVIEW_CAMERAS="${SPACEMOUSE_TELEOP_MULTIVIEW_CAMERAS:-front,side,top}"
MULTIVIEW_LAYOUT="${SPACEMOUSE_TELEOP_MULTIVIEW_LAYOUT:-grid}"
MULTIVIEW_WIDTH="${SPACEMOUSE_TELEOP_MULTIVIEW_WIDTH:-380}"
MULTIVIEW_HEIGHT="${SPACEMOUSE_TELEOP_MULTIVIEW_HEIGHT:-260}"
MULTIVIEW_RATE="${SPACEMOUSE_TELEOP_MULTIVIEW_RATE:-15}"

if [[ -n "${PYTHON_BIN:-}" ]]; then
  PYTHON="$PYTHON_BIN"
elif [[ "$WANT_VIEWER" != "0" && -x ".venv/bin/mjpython" ]]; then
  PYTHON=".venv/bin/mjpython"
elif [[ "$WANT_VIEWER" != "0" ]] && command -v mjpython >/dev/null 2>&1; then
  PYTHON="$(command -v mjpython)"
elif [[ "$WANT_VIEWER" != "0" && "$(uname -s)" == "Darwin" ]]; then
  echo "MuJoCo viewer on macOS must be launched with mjpython." >&2
  echo "Install the sim extras in the venv, then retry: UV_CACHE_DIR=.uv-cache uv pip install -e '.[sim,hardware]'" >&2
  exit 1
elif [[ -x ".venv/bin/python" ]]; then
  PYTHON=".venv/bin/python"
else
  PYTHON="python"
fi

export PYTHONPATH="$ROOT_DIR/src${PYTHONPATH:+:$PYTHONPATH}"

ARGS=(
  scripts/mujoco_teleop.py
  --backend "$BACKEND"
  --axis-convention "$AXIS_CONVENTION"
  --hz "$HZ"
  --config "$CONFIG"
  --target-mode "$TARGET_MODE"
  --arm-control-mode "$ARM_CONTROL_MODE"
  --position-gain "$POSITION_GAIN"
  --orientation-gain "$ORIENTATION_GAIN"
  --print-rate "$PRINT_RATE"
  --log "$LOG_PATH"
)

if [[ "$WANT_VIEWER" != "0" ]]; then
  ARGS+=(--viewer --camera "$CAMERA")
  if [[ "$WANT_MULTIVIEW" != "0" ]]; then
    ARGS+=(
      --multiview
      --multiview-cameras "$MULTIVIEW_CAMERAS"
      --multiview-layout "$MULTIVIEW_LAYOUT"
      --multiview-width "$MULTIVIEW_WIDTH"
      --multiview-height "$MULTIVIEW_HEIGHT"
      --multiview-rate "$MULTIVIEW_RATE"
    )
  fi
fi

echo "Starting MuJoCo teleop: backend=$BACKEND viewer=$WANT_VIEWER python=$PYTHON"
exec "$PYTHON" "${ARGS[@]}" "$@"
