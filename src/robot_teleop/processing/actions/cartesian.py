import numpy as np
from scipy.spatial.transform import Rotation


def rotation(state):
    return Rotation.from_quat(np.asarray(state.quaternion)[[1, 2, 3, 0]])


def encode(a, b, delta):
    p = np.asarray(b.position) - (np.asarray(a.position) if delta else 0)
    R = rotation(b) * rotation(a).inv() if delta else rotation(b)
    return np.r_[p, R.as_rotvec()]


def decode(a, value, delta):
    p = value[:3] + (np.asarray(a.position) if delta else 0)
    R = Rotation.from_rotvec(value[3:6])
    if delta:
        R = R * rotation(a)
    q = R.as_quat()
    return p, q[[3, 0, 1, 2]]
