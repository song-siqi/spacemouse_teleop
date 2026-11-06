"""Explicit replay tracking limits, independent of slow start preparation."""

import numpy as np

DEFAULTS = {"joint_speed_radps": 0.5, "joint_accel_radps2": 1.5}


def joint_limits(settings=None):
    from robot_teleop.robots import ROBOTS

    limits = {**DEFAULTS, **(settings or {})}
    if set(limits) != set(DEFAULTS):
        raise ValueError("Unknown joint replay limit")
    if any(
        isinstance(v, bool) or not np.isfinite(v) or v <= 0 for v in limits.values()
    ):
        raise ValueError("Replay joint limits must be positive finite numbers")
    if limits["joint_speed_radps"] > min(ROBOTS["xarm6"].velocity_limits):
        raise ValueError("Replay speed exceeds robot joint velocity limits")
    return limits


def preflight(states, mode, speed=1.0, limits=None):
    """Necessary sampled-reference checks; does not certify physical tracking."""
    limits = joint_limits(limits)
    if not np.isfinite(speed) or not 0 < speed <= 1:
        raise ValueError("Replay speed must be in (0,1]")
    if "joint" not in mode:
        return dict(
            event="replay_preflight",
            mode=mode,
            speed=speed,
            joint_reference_check="not_applicable",
        )
    t = np.array([s.time for s in states])
    q = np.array([s.joints for s in states])
    if len(t) < 2 or not np.isfinite(q).all() or np.any(np.diff(t) <= 0):
        raise ValueError("Invalid replay reference")
    dt = np.diff(t) / speed
    v = np.diff(q, axis=0) / dt[:, None]
    a = np.diff(v, axis=0) / ((dt[:-1] + dt[1:]) / 2)[:, None]
    peak_v = float(np.max(np.linalg.norm(v, axis=1)))
    peak_a = float(np.max(np.linalg.norm(a, axis=1))) if len(a) else 0.0
    if peak_v > limits["joint_speed_radps"] + 1e-9:
        raise ValueError(
            f"Reference requires {peak_v:.4f} rad/s joint vector speed; replay limit is {limits['joint_speed_radps']:.4f}. No motion started."
        )
    if peak_a > limits["joint_accel_radps2"] + 1e-9:
        raise ValueError(
            f"Reference sampled acceleration requires {peak_a:.4f} rad/s²; replay limit is {limits['joint_accel_radps2']:.4f}. No motion started."
        )
    return dict(
        event="replay_preflight",
        mode=mode,
        speed=speed,
        reference_peak_joint_speed_radps=peak_v,
        reference_peak_sampled_joint_accel_radps2=peak_a,
        limits=limits,
        physical_tracking_verified=False,
    )
