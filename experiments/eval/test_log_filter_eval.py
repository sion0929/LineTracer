import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from log_filter_eval import make_filter  # noqa: E402
from line_follower import vision  # noqa: E402
from line_follower.line_map import C0, RES, LineMap  # noqa: E402
from line_follower.perception.log import PerceptionLog, load  # noqa: E402


class _P:
    def __init__(self, v):
        self.value = v


class Node:
    def __init__(self, p):
        self.p = p

    def has_parameter(self, k):
        return k in self.p

    def get_parameter(self, k):
        return _P(self.p[k])


def _meta():
    return {"grid": {"res_m": vision.RES, "x_near_m": vision.X_NEAR,
                     "x_far_m": vision.X_FAR, "y_half_m": vision.Y_HALF,
                     "shape": [vision.BH, vision.BW]}}


def _col_for_y(y):
    """BEV column whose cell centre is nearest to y (positive = left)."""
    y_half = vision.Y_HALF
    return int(np.argmin(np.abs((y_half - (np.arange(vision.BW) + 0.5) * vision.RES) - y)))


def test_hill_lateral_clears_far_columns_only_on_hills():
    filt = make_filter("hill_lateral_0.20")
    meta = _meta()
    line = np.zeros((vision.BH, vision.BW), bool)
    far_col = _col_for_y(0.30)
    mid_col = _col_for_y(0.0)
    line[:, far_col] = True
    line[:, mid_col] = True
    y_far = vision.Y_HALF - (far_col + 0.5) * vision.RES
    assert abs(y_far) > 0.20

    on_hill = filt(line, 2, meta)
    assert not on_hill[:, far_col].any()
    assert on_hill[:, mid_col].all()

    on_flat = filt(line, 0, meta)
    assert np.array_equal(on_flat, line)
    assert on_flat[:, far_col].all()


def test_rebuild_synthetic_straight_line(tmp_path):
    log = PerceptionLog(Node({}), str(tmp_path))
    line = np.zeros((vision.BH, vision.BW), bool)
    mid = _col_for_y(0.0)
    line[:, mid - 1:mid + 2] = True          # straight line ahead at y = 0
    valid = np.ones_like(line)
    poses = []
    for i in range(30):
        pose = (i * 0.01, 0.0, 0.0)
        poses.append(pose)
        log.record(i / 30.0, pose, 0.0, i * 0.01, True,
                   line.astype(np.uint8) * 255, valid.astype(np.uint8) * 255,
                   slope="flat")
    log.flush()

    _meta_log, d = load(tmp_path)
    line_map = LineMap()
    for i in np.nonzero(d["used"])[0]:
        line_map.update(d["line"][i].astype(np.uint8) * 255,
                        d["valid"][i].astype(np.uint8) * 255, d["pose"][i])

    i, j = np.nonzero(line_map.grid > 0)
    assert len(i) > 0
    x = (j - C0) * RES
    y = (i - C0) * RES
    assert np.abs(y).max() <= 0.01
    assert x.min() >= vision.X_NEAR - 0.02
    assert x.max() >= vision.X_FAR
