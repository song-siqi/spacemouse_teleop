"""Hardware-independent scheduling of reconstructed reference targets."""

import json, time
from dataclasses import replace
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from ..processing.contracts import State
from ..processing.actions.cartesian import rotation
from .reconstruction import reconstruct


def load_reference(path):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(path)
    m = json.loads((root / "processing_manifest.json").read_text())
    if m["status"] != "complete":
        raise ValueError("Incomplete dataset")
    truth = [
        State(**s) for s in json.loads((root / "reference_trajectory.json").read_text())
    ]
    ds = LeRobotDataset("local/teleop", root=root / "lerobot", video_backend="pyav")
    numbers = ds.select_columns(["action"])
    values = np.array([np.asarray(numbers[i]["action"]) for i in range(len(ds))])
    rebuilt = reconstruct(truth[0], values, m["config"]["action_mode"])
    return [replace(a, time=b.time) for a, b in zip(rebuilt, truth)], m["config"][
        "action_mode"
    ]


def run(
    states,
    mode,
    executor,
    *,
    speed=1.0,
    hz=60,
    settle_s=0.3,
    clock=time.monotonic,
    sleep=time.sleep,
    log=lambda row: None,
):
    if (
        not np.isfinite([speed, hz, settle_s]).all()
        or not 0 < speed <= 1
        or hz <= 0
        or settle_s < 0
    ):
        raise ValueError("Invalid replay speed/rate")
    from ..processing.config import MODES

    if (
        mode not in MODES
        or len(states) < 2
        or any(
            b.time <= a.time or len(a.joints) != len(b.joints)
            for a, b in zip(states, states[1:])
        )
    ):
        raise ValueError("Invalid reference trajectory")
    duration = (states[-1].time - states[0].time) / speed
    if hasattr(executor, "preflight"):
        log(executor.preflight(states, mode, speed))
    try:
        executor.connect(states[0], mode)
        start = clock()
        previous = start
        i = 0
        while True:
            now = clock()
            elapsed = now - start
            if now - previous > 0.25:
                raise RuntimeError("Replay scheduler stalled")
            previous = now
            t = states[0].time + min(elapsed, duration) * speed
            while i < len(states) - 2 and states[i + 1].time < t:
                i += 1
            a, b = states[i : i + 2]
            dt = b.time - a.time
            if dt <= 0:
                raise ValueError("Non-increasing reference time")
            w = float(np.clip((t - a.time) / dt, 0, 1))
            q = (1 - w) * np.asarray(a.joints) + w * np.asarray(b.joints)
            p = (1 - w) * np.asarray(a.position) + w * np.asarray(b.position)
            R = Slerp([0, 1], Rotation.concatenate([rotation(a), rotation(b)]))([w])[0]
            quat = R.as_quat()[[3, 0, 1, 2]]
            target = State(
                t,
                q.tolist(),
                p.tolist(),
                quat.tolist(),
                a.gripper + (b.gripper - a.gripper) * w,
                a.gripper_pulse + (b.gripper_pulse - a.gripper_pulse) * w,
            )
            velocity = (
                (np.asarray(b.joints) - a.joints) / dt
                if "joint" in mode
                else np.r_[
                    (np.asarray(b.position) - a.position) / dt,
                    (rotation(b) * rotation(a).inv()).as_rotvec() / dt,
                ]
            ) * speed
            if elapsed >= duration:
                velocity = np.zeros_like(velocity)
            feedback = executor.step(target, velocity, mode, 1 / hz)
            log(dict(elapsed_s=elapsed, reference_time=t, feedback=feedback))
            if elapsed >= duration + settle_s:
                break
            sleep(max(0, 1 / hz - (clock() - now)))
    finally:
        executor.close()
