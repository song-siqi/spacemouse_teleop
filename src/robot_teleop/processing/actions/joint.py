import numpy as np


def encode(a, b, delta):
    return (
        np.asarray(b.joints) - np.asarray(a.joints) if delta else np.asarray(b.joints)
    )


def decode(a, value, delta):
    return np.asarray(a.joints) + value if delta else np.asarray(value)
