"""Clock observations only; no fitted mapping, alignment or latency correction."""
from __future__ import annotations
import threading
import time

class ClockSampler:
    def __init__(self, publish, on_event, interval_s=1., *, monotonic_ns=time.monotonic_ns,
                 realtime_ns=time.time_ns, on_error=None):
        if interval_s <= 0:
            raise ValueError("Clock anchor interval must be positive")
        self.publish, self.on_event = publish, on_event
        self.interval_s, self.monotonic_ns, self.realtime_ns = interval_s, monotonic_ns, realtime_ns
        self.on_error = on_error or (lambda reason: None)
        self.error = None
        self.stop_event = threading.Event()
        self.thread = None
        self.sequence = self.epoch = 0
        self.previous = None

    def sample(self, reason):
        before = self.monotonic_ns()
        realtime = self.realtime_ns()
        after = self.monotonic_ns()
        if after < before:
            raise RuntimeError("Host monotonic clock moved backwards")
        lower, upper = realtime - after, realtime - before
        if self.previous and (lower > self.previous[1] + 5_000_000 or
                              upper < self.previous[0] - 5_000_000):
            self.epoch += 1
            self.on_event({"event": "host_clock_mapping_change", "received_at": after / 1e9,
                           "clock_epoch": self.epoch, "threshold_ns": 5_000_000,
                           "meaning": "offset observation changed; no correction applied"})
        self.previous = lower, upper
        self.sequence += 1
        row = {"sequence": self.sequence, "reason": reason, "clock_id": "host_realtime",
               "clock_epoch": self.epoch, "monotonic_before_ns": before, "realtime_ns": realtime,
               "monotonic_after_ns": after, "read_interval_ns": after - before}
        self.publish(row)
        return row

    def start(self):
        self.sample("session_start")
        self.thread = threading.Thread(target=self._run, name="clock-observations", daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while not self.stop_event.wait(self.interval_s):
                self.sample("periodic")
        except Exception as exc:
            self.error = f"Clock recorder failed: {exc}"
            self.on_error(self.error)

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(2)
        self.sample("session_end")
