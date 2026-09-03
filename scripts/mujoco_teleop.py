#!/usr/bin/env python3
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from spacemouse_teleop.cli.mujoco_teleop import main  # noqa: E402


if __name__ == "__main__":
    main()
