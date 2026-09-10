#!/usr/bin/env bash
set -euo pipefail
COLLECTION_REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$COLLECTION_REPO_DIR/.venv/bin/python" -m spacemouse_teleop.cli.xarm_collect --config "$COLLECTION_REPO_DIR/configs/spacemouse_xarm6_collection.json" "$@"
