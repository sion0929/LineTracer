"""Recognize compact terminal bars and forward branches in a local DL line map."""

import numpy as np


def terminal_shape(points):
    """Return (compact terminal, forward reconnect point) in incoming-line coordinates."""
    along, side = points.T
    # Select a longitudinal corridor where a loop arm crossing sideways is unlikely
    # to give sustained support. A gap between the intersection and this branch is allowed.
    forward = points[(along >= 0.10) & (along <= 0.29) & (abs(side) <= 0.045)]
    if len(forward) >= 8:
        lo, hi = np.quantile(forward[:, 0], [0.15, 0.85])
        if hi - lo >= 0.038:
            near = forward[forward[:, 0] <= lo + 0.026]
            return False, np.median(near, axis=0)
    arm = points[(abs(along) <= 0.045) & (abs(side) >= 0.05)]
    if len(arm) < 10:
        return False, None
    # True terminal bars have approximately 10 cm arms. More than isolated
    # pixels beyond either cap is evidence for a crossing, loop or broad corner.
    ends = np.array([-arm[:, 1].min(), arm[:, 1].max()])
    extended = (abs(side) >= 0.135) & (abs(along) <= 0.10)
    compact = np.all((ends >= 0.067) & (ends <= 0.13)) and np.count_nonzero(extended) < 6
    return bool(compact), None
