from __future__ import annotations

import json
import queue
import threading
import time
from pathlib import Path


class AsyncJsonlRecorder:
    """Bounded queue; serialization and disk flush never run in the motion loop."""
    def __init__(self, path, capacity=2048):
        self.path = Path(path)
        self.queue = queue.Queue(capacity)
        self.error = None
        self._stop = threading.Event()
        self.thread = None

    def open(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("a", encoding="utf-8")
        self.thread = threading.Thread(target=self._run, name="teleop-log", daemon=True)
        self.thread.start()

    def write(self, row):
        if self.error:
            raise RuntimeError(f"Log writer failed: {self.error}")
        try:
            self.queue.put_nowait(row)
        except queue.Full as exc:
            raise RuntimeError("Log queue full; refusing to block control loop") from exc

    def _run(self):
        try:
            flushed = time.monotonic()
            while not self._stop.is_set() or not self.queue.empty():
                try:
                    row = self.queue.get(timeout=.1)
                except queue.Empty:
                    continue
                self.file.write(json.dumps(row, allow_nan=False, sort_keys=True) + "\n")
                if time.monotonic() - flushed >= 1:
                    self.file.flush()
                    flushed = time.monotonic()
        except Exception as exc:
            self.error = exc
        finally:
            self.file.close()

    def close(self):
        self._stop.set()
        if self.thread:
            self.thread.join(2)
            if self.thread.is_alive():
                raise RuntimeError("Log writer did not finish within two seconds")
        if self.error:
            raise RuntimeError(f"Log writer failed: {self.error}")
