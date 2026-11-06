"""Native xArm report interpretation; never initializes the SDK."""

import numpy as np
from scipy.spatial.transform import Rotation


def robot_values(row):
    v = row["values"]["values"]
    q = np.asarray(v["joints_rad"], float)
    tcp = np.asarray(v["tcp_mm_rotvec_rad"], float)
    if q.shape != (6,) or tcp.shape != (6,) or not np.isfinite(np.r_[q, tcp]).all():
        raise ValueError("Malformed native xArm state")
    return q, tcp[:3] / 1000, Rotation.from_rotvec(tcp[3:])


def gripper_values(row, config):
    v = row["values"]
    pulse = float(v["values"]["position_pulse"])
    g = (pulse - config["gripper_open"]) / (
        config["gripper_closed"] - config["gripper_open"]
    )
    if not np.isfinite(g) or not 0 <= g <= 1:
        raise ValueError("Measured gripper outside declared mapping range")
    return g, pulse
