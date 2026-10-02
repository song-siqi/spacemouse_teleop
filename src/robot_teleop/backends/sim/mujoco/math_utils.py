from __future__ import annotations

import math

import numpy as np


def quat_normalize(quat) -> np.ndarray:
    result = np.asarray(quat, dtype=float)
    norm = np.linalg.norm(result)
    if norm <= 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return result / norm


def quat_multiply(a, b) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return quat_normalize(
        np.array(
            [
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ]
        )
    )


def quat_from_rotvec(rotvec) -> np.ndarray:
    rotvec = np.asarray(rotvec, dtype=float)
    angle = np.linalg.norm(rotvec)
    if angle < 1e-12:
        return quat_normalize(np.array([1.0, 0.5 * rotvec[0], 0.5 * rotvec[1], 0.5 * rotvec[2]]))
    axis = rotvec / angle
    half = 0.5 * angle
    return np.array(
        [math.cos(half), axis[0] * math.sin(half), axis[1] * math.sin(half), axis[2] * math.sin(half)]
    )


def quat_to_mat(quat) -> np.ndarray:
    w, x, y, z = quat_normalize(quat)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def orientation_error(desired_quat, current_quat) -> np.ndarray:
    desired = quat_to_mat(desired_quat)
    current = quat_to_mat(current_quat)
    return 0.5 * (
        np.cross(current[:, 0], desired[:, 0])
        + np.cross(current[:, 1], desired[:, 1])
        + np.cross(current[:, 2], desired[:, 2])
    )
