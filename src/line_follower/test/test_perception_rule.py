"""인지 (perception): the rule-based OpenCV threshold plugin `perception/rule.py`.

Synthetic camera frames (`synth.py`: black 2 cm tape on a beige mat, exact camera
model) go through RulePerception.process and the BEV line mask is compared with
the ground truth in vision's BEV grid.

Measured on rp/ros2:jazzy-lab (OpenCV 4.6), the basis of the thresholds below:
  * strict BEV IoU (inside the validity mask): straight 0.80, offset +-8 cm 0.80,
    angled 20 deg 0.86, curve left r=1 m 0.97, curve right r=1 m 0.70, 90 deg corner
    0.80.  A 2 cm line is only 4-5 cells wide, so the half-cell skew of the BEV grid
    (see test_vision KNOWN BUG) costs up to 0.3 IoU on its own.  With a 1-cell
    tolerance precision and recall are 1.00 for every shape.
  * lateral error of the detected centre at x = 0.35 / 0.50 / 0.70 m: <= 5.8 mm
    (curve right at 0.35 m), +2.5 mm for straight lines
  * brightness x0.6 .. x1.3: IoU unchanged except curve left 0.94 at x1.3
  * empty mat: 0 line cells; dark box: 100 % of its footprint reported as line
  * CPU: mean 0.4 ms, p95 0.5 ms per frame (lab PC)
"""
import math
import time

import cv2
import numpy as np
import pytest

import synth
from line_follower import vision
from line_follower.perception.base import PerceptionOutput
from line_follower.perception.rule import RulePerception

RES = vision.RES

SHAPES = {
    'straight': [synth.straight(0.0)],
    'offset_left_8cm': [synth.straight(0.08)],
    'offset_right_8cm': [synth.straight(-0.08)],
    'angled_20deg': [synth.straight(-0.06, 20.0, 0.0, 2.0, x_ref=0.3)],
    'curve_left_r1m': [synth.arc(1.0, 1.6, start=(0.0, -0.1), n=60)],
    'curve_right_r1m': [synth.arc(-1.0, 1.6, start=(0.0, 0.1), n=60)],
    'corner_90_left': [np.array([[0.0, 0.0], [0.6, 0.0], [0.6, 0.6]])],
}
IOU_MIN = 0.60          # measured minimum 0.70
TOL_PR_MIN = 0.97       # measured 1.00
LAT_ERR_MAX = 0.010     # measured maximum 5.8 mm


@pytest.fixture(scope='module')
def plugin():
    return RulePerception(None)


def _process(plugin, polylines, **render_kw):
    bgr, gray = synth.render(polylines, **render_kw)
    return plugin.process(bgr, gray, render_kw.get('pitch', 0.0), vision.X_FAR)


def _iou(det, truth, valid):
    d, t = (det > 0) & valid, (truth > 0) & valid
    return (d & t).sum() / max(1, (d | t).sum())


def _tolerant_pr(det, truth, valid):
    """Precision / recall when a detection within one cell of the truth counts."""
    ker = np.ones((3, 3), np.uint8)
    d, t = (det > 0) & valid, (truth > 0) & valid
    prec = (d & (cv2.dilate(t.astype(np.uint8), ker) > 0)).sum() / max(1, d.sum())
    rec = (t & (cv2.dilate(d.astype(np.uint8), ker) > 0)).sum() / max(1, t.sum())
    return prec, rec


def _centre_y(line, valid, x):
    row = int(round((vision.X_FAR - x) / RES))
    cols = np.nonzero((line[row] > 0) & (valid[row] > 0))[0]
    return None if cols.size == 0 else vision.bev_to_robot(cols.mean(), row)[1]


def _check_shape(out, polylines, name):
    valid = out.valid > 0
    truth = synth.truth_bev(polylines)
    iou = _iou(out.line, truth, valid)
    prec, rec = _tolerant_pr(out.line, truth, valid)
    errs = {}
    for x in (0.35, 0.5, 0.7):
        y_true = synth.lateral_at(polylines[0], x)
        y_det = _centre_y(out.line, out.valid, x)
        if y_true is None:
            assert y_det is None, f'{name}: line reported at x={x} where there is none'
            continue
        assert y_det is not None, f'{name}: nothing detected at x={x}'
        errs[x] = y_det - y_true
    lateral = ' '.join(f'x={x}:{e * 1000:+.1f}mm' for x, e in errs.items())
    print(f'{name}: IoU {iou:.3f}  P/R(1 cell) {prec:.3f}/{rec:.3f}  lateral err {lateral}')
    assert iou >= IOU_MIN
    assert prec >= TOL_PR_MIN and rec >= TOL_PR_MIN
    assert all(abs(e) <= LAT_ERR_MAX for e in errs.values()), errs
    return errs


def test_output_contract(plugin):
    out = _process(plugin, SHAPES['straight'])
    assert isinstance(out, PerceptionOutput)
    for mask in (out.line, out.valid):
        assert mask.shape == (vision.BH, vision.BW) and mask.dtype == np.uint8
        assert set(np.unique(mask)) <= {0, 255}
    assert out.obstacle is None
    assert not (out.line > 0)[out.valid == 0].any(), 'line reported outside the seen area'


@pytest.mark.parametrize('name', list(SHAPES))
def test_line_shapes_match_ground_truth(plugin, name):
    _check_shape(_process(plugin, SHAPES[name]), SHAPES[name], name)


def test_corner_bar_is_found_at_the_right_distance(plugin):
    out = _process(plugin, SHAPES['corner_90_left'])
    col = int(round((vision.Y_HALF - 0.25) / RES))          # bar column at y = +25 cm
    rows = np.nonzero(out.line[:, col] > 0)[0]
    assert rows.size
    assert vision.bev_to_robot(col, rows.mean())[0] == pytest.approx(0.6, abs=0.01)
    # nothing on the right of the corner (y < -2 cm) and nothing beyond the bar
    right = int(round((vision.Y_HALF + 0.02) / RES))
    assert not out.line[:, right:].any()
    assert not out.line[:int(round((vision.X_FAR - 0.62) / RES))].any()


@pytest.mark.parametrize('gain', [0.6, 0.8, 1.0, 1.3])
@pytest.mark.parametrize('name', ['straight', 'curve_left_r1m', 'corner_90_left'])
def test_global_brightness_change_still_works(plugin, name, gain):
    _check_shape(_process(plugin, SHAPES[name], gain=gain), SHAPES[name], f'{name} x{gain}')


@pytest.mark.parametrize('gain', [0.6, 1.0, 1.3])
def test_empty_mat_gives_no_line(plugin, gain):
    out = _process(plugin, [], gain=gain)
    seen = (out.valid > 0).sum()
    assert seen > 11000
    assert (out.line > 0).sum() <= 0.001 * seen      # measured 0


def test_pitch_argument_is_used_for_the_projection(plugin):
    bar = [np.array([[0.6, -0.3], [0.6, 0.3]])]
    th = math.radians(2.0)
    bgr, gray = synth.render(bar, pitch=th)
    col = vision.BW // 2
    for assumed, expect in ((th, 0.6), (0.0, 0.52)):
        rows = np.nonzero(plugin.process(bgr, gray, assumed, vision.X_FAR).line[:, col] > 0)[0]
        assert vision.bev_to_robot(col, rows.mean())[0] == pytest.approx(expect, abs=0.0075)


@pytest.mark.xfail(strict=True, reason='KNOWN BUG: obstacles are reported as line (obstacle=None)')
def test_dark_box_is_not_reported_as_line(plugin):
    box = (0.50, 0.60, -0.05, 0.05, 0.10)            # 10x10x10 cm dark box 0.5 m ahead
    out = _process(plugin, [synth.straight(0.25)], boxes=[box])
    rows = slice(int(round((vision.X_FAR - 0.60) / RES)), int(round((vision.X_FAR - 0.50) / RES)))
    cols = slice(int(round((vision.Y_HALF - 0.05) / RES)), int(round((vision.Y_HALF + 0.05) / RES)))
    line_frac = (out.line[rows, cols] > 0).mean()
    print(f'dark box footprint reported as line: {line_frac:.0%}, obstacle={out.obstacle}')
    assert line_frac < 0.05
    assert out.obstacle is not None and out.obstacle[rows, cols].any()


def test_cpu_latency_per_frame(plugin):
    bgr, gray = synth.render(SHAPES['curve_left_r1m'])
    for _ in range(10):
        plugin.process(bgr, gray, 0.0, vision.X_FAR)
    lat = []
    for _ in range(100):
        t0 = time.perf_counter()
        plugin.process(bgr, gray, 0.0, vision.X_FAR)
        lat.append(time.perf_counter() - t0)
    lat = np.array(lat) * 1000.0
    mean, p95 = lat.mean(), np.percentile(lat, 95)
    print(f'rule perception latency: mean {mean:.2f} ms, p95 {p95:.2f} ms (640x480 -> 220x108 BEV)')
    assert p95 < 60.0
