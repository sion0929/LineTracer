"""Small GT helpers for the eval scripts (dev only)."""
import numpy as np
from scipy.spatial import cKDTree


def turn_mask(pts, centre, win=0.25, deg=20):
    """True for points near a part of the centre polyline whose heading changes > deg within +-win m."""
    c = np.asarray(centre, float)
    seg = np.diff(c, axis=0); arc = np.r_[0, np.cumsum(np.linalg.norm(seg, axis=1))]
    h = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0])); h = np.r_[h, h[-1]]
    turn = np.zeros(len(c), bool)
    for i in range(len(c)):
        m = np.abs(arc - arc[i]) <= win
        turn[i] = np.ptp(h[m]) > np.radians(deg)
    dense, flag = [], []
    for i in range(len(c) - 1):
        n = max(1, int(np.linalg.norm(c[i + 1] - c[i]) / 0.01))
        for k in range(n):
            dense.append(c[i] + (c[i + 1] - c[i]) * k / n); flag.append(turn[i] or turn[i + 1])
    return np.array(flag)[cKDTree(np.array(dense)).query(pts)[1]]
