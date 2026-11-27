"""Offline geometry test for perception/depth.py (no rclpy, no real ONNX inference).

The network output is replaced with one synthetic line pixel per known ground point:
the pixel comes from vision.ground_to_image, its depth is that point's camera-axis
depth, and process() must place it back on the BEV grid within 1 cm.

Run:
    /home/rvlab/miniforge3/envs/lf_b_dl/bin/python -m pytest experiments/tests/test_depth_geometry.py -q
"""
import math
import sys
import types
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "line_follower"))

from line_follower import vision                                    # noqa: E402
from line_follower.perception.depth import (                        # noqa: E402
    DepthPerception, W_IN, H_IN, REST_PITCH)


class _Param:
    def __init__(self, value):
        self.value = value


class _FakeNode:
    def __init__(self, model_path):
        self.values = {
            "depth.model_path": model_path,
            "depth.min_logit_margin": 0.0,
            "depth.max_depth": 1.4,
        }

    def declare_parameter(self, name, default):
        self.values.setdefault(name, default)

    def get_parameter(self, name):
        return _Param(self.values[name])


class _FakeSession:
    def __init__(self, *args, **kwargs):
        pass


def _fake_onnxruntime():
    fake = types.SimpleNamespace()
    fake.get_available_providers = lambda: ["CPUExecutionProvider"]
    fake.SessionOptions = lambda: types.SimpleNamespace()
    fake.InferenceSession = lambda *a, **k: _FakeSession()
    return fake


def _make_depth_perception(monkeypatch, tmp_path):
    """Instantiate the real __init__ (which builds the pixel-ray tables) without a model."""
    model = tmp_path / "dummy.onnx"
    model.write_bytes(b"")
    monkeypatch.setitem(sys.modules, "onnxruntime", _fake_onnxruntime())
    return DepthPerception(_FakeNode(str(model)))


def _fake_network_output(points):
    """One line pixel + camera-axis depth for each ground point, on the 320x240 grid."""
    logits = np.zeros((2, H_IN, W_IN), np.float32)
    depth = np.zeros((H_IN, W_IN), np.float32)
    c, s = math.cos(REST_PITCH), math.sin(REST_PITCH)
    for x, y in points:
        u, v = vision.ground_to_image(x, y, REST_PITCH)
        # inverse of depth.py's pixel -> full-resolution mapping:
        #   u = (i + 0.5) * vision.W / W_IN - 0.5
        i = int(round((u + 0.5) * (W_IN / vision.W) - 0.5))
        j = int(round((v + 0.5) * (H_IN / vision.H) - 0.5))
        assert 0 <= i < W_IN and 0 <= j < H_IN
        z_cam = (x - vision.CAM_X) * c - vision.CAM_H * s   # camera-axis depth
        logits[1, j, i] = 5.0
        logits[0, j, i] = 0.0
        depth[j, i] = np.float32(z_cam)
    return logits, depth


def test_depth_places_line_on_flat_ground(monkeypatch, tmp_path):
    perc = _make_depth_perception(monkeypatch, tmp_path)
    assert perc.ru.shape == (W_IN,)
    assert perc.rv.shape == (H_IN,)

    points = [(0.6, 0.1), (0.5, -0.2), (0.7, 0.0), (0.55, 0.25)]
    logits, depth = _fake_network_output(points)
    perc.infer = lambda bgr: (logits, depth)

    bgr = np.zeros((vision.H, vision.W, 3), np.uint8)
    gray = np.zeros((vision.H, vision.W), np.uint8)
    out = perc.process(bgr, gray, REST_PITCH, 1.4)

    assert out.line.shape == (vision.BH, vision.BW)
    assert int((out.line > 0).sum()) >= len(points)

    for x, y in points:
        col = int(round((vision.Y_HALF - y) / vision.RES))
        row = int(round((vision.X_FAR - x) / vision.RES))
        assert 4 <= col < vision.BW - 4 and 4 <= row < vision.BH - 4

        # The output is dilated by 2 cells; measure the centroid of the isolated blob
        # (the synthetic points are >8 cells apart) and decode it back to metres.
        win = out.line[row - 4:row + 5, col - 4:col + 5]
        rr, cc = np.nonzero(win)
        assert rr.size > 0, f"no line pixel near expected ({x},{y}) at row={row} col={col}"
        row_c = row - 4 + float(rr.mean())
        col_c = col - 4 + float(cc.mean())
        got_x = vision.X_FAR - row_c * vision.RES
        got_y = vision.Y_HALF - col_c * vision.RES
        err = math.hypot(got_x - x, got_y - y)
        assert err <= 0.01, (
            f"point ({x},{y}) came back as ({got_x:.4f},{got_y:.4f}), "
            f"error {err * 100:.2f} cm > 1 cm")
