from __future__ import annotations

import json
from pathlib import Path
from typing import Mapping, Optional

from spacemouse_teleop.spacemouse.command import RawSpaceMouseState, TeleopCommand


class JsonlRecorder:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._file = None

    def open(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("a", encoding="utf-8")

    def write(
        self,
        raw: RawSpaceMouseState,
        command: TeleopCommand,
        mode: str,
        note: Optional[str] = None,
        extras: Optional[Mapping[str, object]] = None,
    ) -> None:
        if self._file is None:
            raise RuntimeError("recorder is not open")
        row = {
            "mode": mode,
            "raw": raw.to_dict(),
            "command": command.to_dict(),
        }
        if note:
            row["note"] = note
        if extras:
            row.update(extras)
        self._file.write(json.dumps(row, sort_keys=True) + "\n")
        self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
        self._file = None

    def __enter__(self) -> "JsonlRecorder":
        self.open()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()
