import numpy as np
from dataclasses import replace
from ..processing.actions import joint, cartesian


def actions(states, mode):
    codec = joint if "joint" in mode else cartesian
    return np.asarray(
        [
            np.r_[codec.encode(a, b, mode.startswith("delta_")), b.gripper]
            for a, b in zip(states, states[1:])
        ],
        dtype=np.float32,
    )


def reconstruct(initial, values, mode):
    states = [initial]
    for v in values:
        a = states[-1]
        if "joint" in mode:
            b = replace(
                a,
                joints=joint.decode(
                    a, np.asarray(v[:-1]), mode.startswith("delta_")
                ).tolist(),
                gripper=float(v[-1]),
            )
        else:
            p, q = cartesian.decode(a, np.asarray(v[:-1]), mode.startswith("delta_"))
            b = replace(
                a, position=p.tolist(), quaternion=q.tolist(), gripper=float(v[-1])
            )
        states.append(b)
    return states
