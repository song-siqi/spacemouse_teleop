import numpy as np


def nearest(times, t):
    j = int(np.searchsorted(times, t))
    return min((max(0, j - 1), min(len(times) - 1, j)), key=lambda i: abs(times[i] - t))


def bracket(times, t, max_gap):
    j = int(np.searchsorted(times, t, side="right"))
    i = j - 1
    if i < 0 or j >= len(times):
        raise ValueError("No state interpolation bracket")
    gap = times[j] - times[i]
    if gap > max_gap:
        raise ValueError("Robot interpolation gap exceeds threshold")
    return i, j, float((t - times[i]) / gap)
