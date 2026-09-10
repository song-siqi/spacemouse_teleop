"""Independent, read-only state acquisition. No motion initialization is allowed here."""
from __future__ import annotations

import math
import threading
import time
from collections import defaultdict

from spacemouse_teleop.backends.xarm_sdk.transport import SDKChannel


class StatePoller:
    def __init__(self, ip, on_sample, on_error, hz=30., timeout=.05,
                 channel_factory=SDKChannel, gripper_hz=0.):
        if not math.isfinite(hz) or hz <= 0 or not math.isfinite(gripper_hz) or gripper_hz < 0:
            raise ValueError("State frequencies must be finite and positive (gripper may be zero)")
        self.ip, self.hz, self.timeout = ip, hz, timeout
        self.on_sample, self.on_error = on_sample, on_error
        self.channel_factory, self.gripper_hz = channel_factory, gripper_hz
        self.stop_event = threading.Event()
        self.threads, self.channels = [], []
        self.sequences = defaultdict(int)

    def start(self):
        targets = [("robot-state", self.hz, False)]
        if self.gripper_hz:
            targets.append(("gripper-state-readonly", self.gripper_hz, True))
        try:
            for name, hz, gripper in targets:
                channel = self.channel_factory(self.ip, self.timeout, writable=False)
                self.channels.append(channel)
                thread = threading.Thread(target=self._run, args=(channel, hz, gripper), name=name, daemon=True)
                self.threads.append(thread)
                thread.start()
        except Exception:
            self.close()
            raise

    def _query(self, channel, kind):
        started = time.monotonic()
        self.sequences[kind] += 1
        sample = {"kind": kind, "sequence": self.sequences[kind], "query_started_at": started,
                  "source": "independent_readonly_query", "device_sequence": None, "valid": False}
        try:
            if kind == "gripper":
                values = channel.gripper_status()
                if not math.isfinite(values["position_pulse"]):
                    raise ValueError("Nonfinite gripper position")
            else:
                result = channel.query("get_joint_pos" if kind == "joints" else "get_tcp_pose")
                if len(result) != (7 if kind == "joints" else 6):
                    raise ValueError(f"Unexpected {kind} response length: {len(result)}")
                if not all(math.isfinite(v) for v in result):
                    raise ValueError(f"Nonfinite {kind} response")
                values = {"joints_rad" if kind == "joints" else "tcp_mm_rad": list(result[:6])}
            sample.update(values=values, valid=True)
        except Exception as exc:
            sample.update(values={}, error=str(exc))
        received = time.monotonic()
        sample.update(query_finished_at=received, received_at=received)
        self.on_sample(sample)
        if not sample["valid"]:
            self.on_error({"source": kind, "reason": sample["error"], "received_at": received})

    def _run(self, channel, hz, gripper):
        tick = time.monotonic()
        try:
            while not self.stop_event.is_set():
                for kind in (("gripper",) if gripper else ("joints", "tcp")):
                    if self.stop_event.is_set():
                        break
                    self._query(channel, kind)
                tick += 1 / hz
                now = time.monotonic()
                if tick < now:
                    tick = now + 1 / hz
                self.stop_event.wait(max(0., tick - now))
        except Exception as exc:
            self.on_error({"source": "state_worker", "reason": str(exc), "received_at": time.monotonic()})

    def close(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join(4 * self.timeout + .2)
        for channel in self.channels:
            channel.close()
        for thread in self.threads:
            thread.join(.2)
        if any(thread.is_alive() for thread in self.threads):
            raise RuntimeError("Read-only state worker did not stop")


class ObservedChannel:
    """A transparent observer; recorder failures never alter SDK call results."""
    def __init__(self, channel, publish, next_sequence):
        self.channel, self.publish, self.next_sequence = channel, publish, next_sequence

    def __getattr__(self, name):
        return getattr(self.channel, name)

    def gripper_status(self):
        started = time.monotonic()
        result = self.channel.gripper_status()
        try:
            self.publish({"kind": "gripper", "sequence": self.next_sequence(), "valid": True,
                          "source": "existing_gripper_worker", "device_sequence": None,
                          "query_started_at": started, "query_finished_at": result["received_at"],
                          "received_at": result["received_at"], "values": dict(result)})
        except Exception:
            # The collection sink owns fault reporting, never motion control.
            pass
        return result
