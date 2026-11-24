"""인지 (perception) on REAL Gazebo frames: the `rule` plugin on a recorded official-track fixture.

No Gazebo and no ROS graph are needed: the frames and the ground truth are stored in
test/data/official_track_rec.npz (28 frames, 0.35 MB), recorded with the development tool
experiments/eval/record_perception.py (set_pose teleports along the official centreline,
lateral +-3 cm, heading +-8 deg):

    python3 experiments/eval/record_perception.py \
        --quota straight=6,corner=6,crossing=5,loop=4,hill=7 --seed 7 \
        --out src/line_follower/test/data/official_track_rec.npz

Scene types (primary): straight 6, corner 6, crossing 5, loop 4 (the official bulb), hill 7
(climbing, on the deck, descending; body pitch -14.5 .. +7.8 deg).  Per frame the fixture holds
the PNG camera frame, the true model pose, the true VISIBLE line rasterised in vision's BEV grid
(gt_bev) and the true route position at x = 0.35 / 0.50 / 0.70 m ahead (probe_y).

The plugin is called exactly as FollowerNode.on_image calls it for a static robot: on a slope
(|IMU pitch| > 3 deg) pitch = REST_PITCH and x_max = 0.50 m, on flat ground pitch = REST_PITCH
and x_max = vision.X_FAR.  The same metrics as experiments/eval/perception_bench.py:
  * BEV IoU: plugin line vs true visible line, both inside the plugin's validity mask (pooled)
  * lateral error p95 over all probes: route position vs the nearest detected run centre in
    that BEV row (+-1 row), matched within 10 cm
  * detection rate: visible route probes with a detected run within 10 cm
  * latency: plugin.process() per frame, 1 OpenCV thread

Measured on rp/ros2:jazzy-lab (OpenCV 4.6, 2026-11-25), the basis of the thresholds:
  IoU 0.296 (flat 0.292, hill 0.326), lateral p95 1.7 cm (p50 0.3 cm), detection 91.7 %
  (55/60 probes), latency mean 0.30 ms / p95 0.32 ms.
Why the IoU is low although the line is found: the 2 cm line is 4 cells wide, so the
~1-row along-track bias (BEV grid scale 5.05 mm/row vs RES 5 mm, CAM_H 0.103 vs 0.102 m in
Gazebo, REST_PITCH 0.33 vs 0.1 deg static pitch) costs about half of the overlap, and two
corner frames see the dark side faces of the hill, which the threshold reports as line.
Thresholds: IoU >= 0.25, lateral p95 <= 2.5 cm, detection >= 0.85, latency p95 < 60 ms.
"""
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import pytest

from line_follower import vision
from line_follower.perception import make_perception

try:
    from line_follower.follower_node import REST_PITCH
except ImportError:                     # no rclpy: same constant as follower_node.py
    REST_PITCH = math.radians(0.33)

DATA = Path(__file__).resolve().parent / 'data' / 'official_track_rec.npz'
HILL_PITCH = math.radians(3.0)          # follower_node: |pitch_f| > 3 deg -> rest pitch, near range
HILL_X_MAX = 0.50
MATCH_GATE = 0.10

MIN_IOU = 0.25
MAX_LATERAL_P95_M = 0.025
MIN_DETECTION = 0.85
MAX_LATENCY_P95_MS = 60.0


class _Node:
    """Minimal FollowerNode stand-in (parameters + filtered IMU pitch) for the plugins."""

    def __init__(self):
        self.params = {}
        self.pitch_f = 0.0

    def declare_parameter(self, name, value=None, descriptor=None):
        self.params[name] = value

    def get_parameter(self, name):
        return type('P', (), {'value': self.params[name]})()


def _node_pitch(pitch_up):
    if abs(pitch_up) > HILL_PITCH:
        return REST_PITCH, HILL_X_MAX
    return REST_PITCH, vision.X_FAR


def _runs(mask_1d):
    edges = np.flatnonzero(np.diff(np.concatenate(([0], mask_1d.astype(np.int8), [0]))))
    return [(a + b - 1) / 2.0 for a, b in zip(edges[::2], edges[1::2])]


def _lateral(pred, probe_x, probe_y):
    """Per probe: None if the route is not visible there, else the error [m] or inf (miss)."""
    out = []
    for x0, y_gt in zip(probe_x, probe_y):
        if not np.isfinite(y_gt):
            out.append(None)
            continue
        r0 = int(round((vision.X_FAR - x0) / vision.RES))
        band = pred[max(r0 - 1, 0):r0 + 2].any(0)
        errs = [abs(vision.Y_HALF - c * vision.RES - y_gt) for c in _runs(band)]
        best = min(errs) if errs else math.inf
        out.append(best if best <= MATCH_GATE else math.inf)
    return out


@pytest.fixture(scope='module')
def recording():
    assert DATA.is_file(), f'missing fixture {DATA}'
    d = np.load(DATA)
    meta = json.loads(str(d['meta']))
    grid = meta['bev']
    if (tuple(d['bev_shape']) != (vision.BH, vision.BW) or grid['res'] != vision.RES
            or grid['x_far'] != vision.X_FAR or grid['y_half'] != vision.Y_HALF):
        pytest.fail(f'fixture was recorded for BEV grid {grid}; vision.py changed it. Re-record it with '
                    'experiments/eval/record_perception.py (command in this module docstring).')
    return d, meta


@pytest.fixture(scope='module')
def measured(recording):
    d, _ = recording
    plugin = make_perception('rule', _Node())
    threads = cv2.getNumThreads()
    cv2.setNumThreads(1)
    try:
        n = len(d['poses'])
        inter = union = 0
        per_split = {'flat': [0, 0], 'hill': [0, 0]}
        lat, ms = [], []
        for i in range(n):
            a, b = d['frames_off'][i:i + 2]
            bgr = cv2.imdecode(d['frames_png'][a:b], cv2.IMREAD_COLOR)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            pitch_up = -float(d['poses'][i][4])           # Gazebo pitch > 0 is nose down
            plugin.node.pitch_f = pitch_up
            pitch, x_max = _node_pitch(pitch_up)
            if i == 0:
                plugin.process(bgr, gray, pitch, x_max)   # warm-up
            t0 = time.perf_counter()
            out = plugin.process(bgr, gray, pitch, x_max)
            ms.append(1000.0 * (time.perf_counter() - t0))
            assert out.line.shape == out.valid.shape == (vision.BH, vision.BW)
            valid = out.valid > 0
            pred = (out.line > 0) & valid
            gt = np.unpackbits(d['gt_bev'][i])[:vision.BH * vision.BW].reshape(vision.BH, vision.BW).astype(bool)
            truth = gt & valid
            fi, fu = int((pred & truth).sum()), int((pred | truth).sum())
            inter, union = inter + fi, union + fu
            split = per_split['hill' if d['hill'][i] else 'flat']
            split[0] += fi
            split[1] += fu
            lat += _lateral(pred, d['probe_x'], d['probe_y'][i])
    finally:
        cv2.setNumThreads(threads)
    visible = [e for e in lat if e is not None]
    hits = [e for e in visible if math.isfinite(e)]
    return dict(
        frames=n, iou=inter / max(union, 1),
        iou_flat=per_split['flat'][0] / max(per_split['flat'][1], 1),
        iou_hill=per_split['hill'][0] / max(per_split['hill'][1], 1),
        lateral_p50=float(np.percentile(hits, 50)) if hits else math.inf,
        lateral_p95=float(np.percentile(hits, 95)) if hits else math.inf,
        detection=len(hits) / max(len(visible), 1), probes=len(visible), detected=len(hits),
        ms_mean=float(np.mean(ms)), ms_p95=float(np.percentile(ms, 95)))


def test_print_measured_numbers(measured, capsys):
    m = measured
    with capsys.disabled():
        print(f"\n[perception recording] rule on {m['frames']} official-track frames: "
              f"IoU {m['iou']:.3f} (flat {m['iou_flat']:.3f}, hill {m['iou_hill']:.3f}), "
              f"lateral p50 {100 * m['lateral_p50']:.2f} cm p95 {100 * m['lateral_p95']:.2f} cm, "
              f"detection {100 * m['detection']:.1f}% ({m['detected']}/{m['probes']}), "
              f"latency mean {m['ms_mean']:.2f} ms p95 {m['ms_p95']:.2f} ms")


def test_fixture_covers_all_scene_types(recording):
    d, meta = recording
    assert meta['map'] == 'line_track'
    types = set(d['category'].tolist())
    assert {'straight', 'corner', 'crossing', 'loop', 'hill'} <= types, types
    assert 25 <= len(d['poses']) <= 30
    assert d['hill'].sum() >= 5
    assert DATA.stat().st_size < 2_000_000


def test_rule_bev_iou(measured):
    assert measured['iou'] >= MIN_IOU, measured


def test_rule_lateral_error_p95(measured):
    assert measured['lateral_p95'] <= MAX_LATERAL_P95_M, measured


def test_rule_detection_rate(measured):
    assert measured['probes'] >= 40, measured
    assert measured['detection'] >= MIN_DETECTION, measured


def test_rule_latency(measured):
    assert measured['ms_p95'] < MAX_LATENCY_P95_MS, measured
