import importlib.util
from pathlib import Path

import numpy as np

from line_follower import vision
from line_follower.perception.log import PerceptionLog

_spec = importlib.util.spec_from_file_location(
    'perception_log_tool', Path(__file__).resolve().parents[3] / 'experiments' / 'eval' / 'perception_log_tool.py')
tool = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tool)


class _Node:
    def has_parameter(self, k):
        return False

    def get_parameter(self, k):
        raise KeyError(k)


def _make_log(path, n=40):
    """Robot drives +x at 1 cm/frame along y=0.2; a line ahead along the world line y=0.2 (BEV: ahead strip)."""
    log = PerceptionLog(_Node(), str(path))
    res = vision.RES
    for i in range(n):
        px = 0.01 * i
        line = np.zeros((vision.BH, vision.BW), np.uint8)
        # world line y = 0.2 -> robot-frame y = 0.2 (yaw 0, robot at y=0): col = (Y_HALF - 0.2)/res - 0.5
        c = int(round((vision.Y_HALF - 0.2) / res - 0.5))
        line[:, c] = 255
        log.record(i / 30, (px, 0.0, 0.0), 0.01 * (i % 3), px, True, line, np.full_like(line, 255))
    log.flush()
    return n


def test_summary_counts(tmp_path):
    n = _make_log(tmp_path)
    s = tool.summarize(tmp_path)
    assert s['frames'] == n and s['used'] == n
    assert abs(s['distance_m'] - 0.01 * (n - 1)) < 1e-9
    assert s['line_cells'] == vision.BH and s['no_line'] == 0
    assert 'frames' in tool.summary_table([tmp_path])


def test_rebuild_places_line(tmp_path):
    _make_log(tmp_path)
    meta, d = tool.load_log(tmp_path)
    hits, seen, (x0, y0), res = tool.accumulate(meta, d)
    rows, cols = np.nonzero(hits)
    ys = y0 + (cols + 0.5) * res
    assert np.all(np.abs(ys - 0.2) <= res * 1.01)
    xs = x0 + (rows + 0.5) * res
    assert xs.min() <= vision.X_NEAR + 0.01 and xs.max() >= vision.X_FAR + 0.39 - 0.01
    assert (seen >= hits).all() and seen.sum() > hits.sum()
    img = tool.render(hits, seen, (x0, y0), res)
    assert img.ndim == 3 and (img == 0).any()
