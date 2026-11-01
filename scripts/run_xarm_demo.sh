#!/usr/bin/env bash
set -euo pipefail

DEMO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$DEMO_ROOT"

if [[ ! -x "$DEMO_ROOT/.venv/bin/python" ]]; then
  echo "Missing project .venv; install the hardware and xarm extras first." >&2
  exit 1
fi

mkdir -p logs
DEMO_STAMP="$(date -u +%Y%m%dT%H%M%SZ)_$$"
DEMO_LOG="$DEMO_ROOT/logs/xarm_demo_${DEMO_STAMP}.jsonl"
ln -sfn "$(basename "$DEMO_LOG")" logs/xarm_demo_latest.jsonl

echo "xArm demo: project .venv; independent configs/spacemouse_xarm6_demo.json"
echo "Default duration: until Ctrl-C. H in this terminal opens the gripper and returns home."
echo "Startup keeps the current pose and gripper position; center all axes and release both buttons to begin."
echo "If waiting for the first HID report, briefly move and release the knob once. Ctrl-C stops."
echo "Log: $DEMO_LOG"

export PYTHONPATH="$DEMO_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"

exec "$DEMO_ROOT/.venv/bin/python" -u -m robot_teleop.cli.xarm_demo \
  --config "$DEMO_ROOT/configs/spacemouse_xarm6_demo.json" \
  --duration 0 --log "$DEMO_LOG" "$@"
