"""CPU tests for rule max-width filtering and hybrid perception (no ROS, no model)."""
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from line_follower import vision
from line_follower.perception import hybrid as hybrid_mod
from line_follower.perception import make_perception
from line_follower.perception.base import PerceptionOutput
from line_follower.perception.hybrid import HybridPerception
from line_follower.perception.rule import RulePerception

MAX_WIDTH = 0.04          # above the 2 cm line (+ raster slack), well below the 20 cm blob


class FakeNode:
    def __init__(self, params=None):
        self.params = dict(params or {})

    def has_parameter(self, name):
        return name in self.params

    def get_parameter(self, name):
        if name not in self.params:
            raise KeyError(name)
        return SimpleNamespace(value=self.params[name])


class StubPerception:
    def __init__(self, line, valid):
        self.line = line
        self.valid = valid

    def process(self, bgr, gray, pitch, x_max):
        return PerceptionOutput(line=self.line.copy(), valid=self.valid.copy())


def _empty():
    return np.zeros((vision.BH, vision.BW), np.uint8)


def _vertical(start, width=4):
    mask = _empty()
    mask[:, start:start + width] = 255
    return mask


def test_max_width_zero_is_bit_identical_to_the_rule_mask():
    gray = np.full((vision.H, vision.W), 200, np.uint8)
    for x in np.arange(vision.X_NEAR + 0.05, vision.X_FAR - 0.02, 0.005):
        u, v = vision.ground_to_image(x, 0.0)     # a dark line on the ground ahead
        u, v = int(round(u)), int(round(v))
        gray[max(0, v - 1):v + 2, max(0, u - 1):u + 2] = 20
    node = FakeNode({'near_scale': 1.0, 'rule.max_width_m': 0.0})
    out = RulePerception(node).process(None, gray, 0.0, vision.X_FAR)
    line, valid = vision.line_mask(gray, 0.0, vision.X_FAR, scale=1.0)
    assert np.array_equal(out.line, line)
    assert np.array_equal(out.valid, valid)
    assert out.obstacle is None
    assert line.any()


def test_remove_wide_keeps_two_cm_line_plus_cross_t_and_curve():
    straight = _vertical(100)
    plus = _empty()
    plus[54:58, 20:180] = 255                     # 4-cell (2 cm) horizontal arm
    plus[10:100, 98:102] = 255                    # 4-cell (2 cm) vertical arm
    tee = _empty()
    tee[54:58, 20:180] = 255
    tee[10:54, 98:102] = 255
    curve = _empty()
    for row in range(8, vision.BH - 8):
        col = int(round(vision.BW / 2 + 32 * np.sin(row / vision.BH * 2 * np.pi)))
        cv2.circle(curve, (col, row), 2, 255, -1)  # ~2 cm wide curve
    blob = _empty()
    blob[30:70, 20:60] = 255                      # 40 cells = 20 cm

    for name, mask in (('straight', straight), ('plus', plus), ('t', tee), ('curve', curve)):
        out = vision.remove_wide(mask, MAX_WIDTH)
        assert np.array_equal(out, mask), f'{name} should survive the thickness filter'

    assert not vision.remove_wide(blob, MAX_WIDTH).any()


def test_remove_wide_off_returns_the_mask_unchanged():
    blob = _empty()
    blob[30:70, 20:60] = 255
    assert np.array_equal(vision.remove_wide(blob, 0.0), blob)


def test_rule_perception_applies_max_width_to_bev_mask(monkeypatch):
    blob = _empty()
    blob[30:70, 20:60] = 255
    valid = np.full_like(blob, 255)
    monkeypatch.setattr(
        'line_follower.perception.rule.vision.line_mask',
        lambda *_args, **_kwargs: (blob.copy(), valid.copy()))
    node = FakeNode({'near_scale': 1.0, 'rule.max_width_m': MAX_WIDTH})
    out = RulePerception(node).process(None, None, 0.0, vision.X_FAR)
    assert not out.line.any()
    assert np.array_equal(out.valid, valid)


def _hybrid(mode, gate_px, rule_line, depth_line, rule_valid=None):
    sensor = HybridPerception.__new__(HybridPerception)
    sensor.node = FakeNode()
    sensor.mode, sensor.gate_px = mode, gate_px
    rule_valid = np.full_like(rule_line, 255) if rule_valid is None else rule_valid
    sensor.rule = StubPerception(rule_line, rule_valid)
    sensor.depth = StubPerception(depth_line, np.full_like(depth_line, 255))
    return sensor, rule_valid


def test_hybrid_and_keeps_only_the_overlap_with_rule_valid():
    rule = _vertical(20) | _vertical(100)
    depth = _vertical(100) | _vertical(160)
    sensor, rule_valid = _hybrid('and', 2, rule, depth)
    out = sensor.process(None, None, 0.0, vision.X_FAR)
    expected = _vertical(100)
    assert np.array_equal(out.line, expected)
    assert np.array_equal(out.valid, rule_valid)
    assert out.obstacle is None


def test_hybrid_rule_gated_keeps_near_depth_components_only():
    # Rule components: far (20), on the depth line (100), and one cell-gap away (105).
    rule = _vertical(20) | _vertical(100) | _vertical(105)
    depth = _vertical(100)
    sensor, rule_valid = _hybrid('rule_gated', 2, rule, depth)
    out = sensor.process(None, None, 0.0, vision.X_FAR)
    expected = _vertical(100) | _vertical(105)
    assert np.array_equal(out.line, expected)
    assert np.array_equal(out.valid, rule_valid)

    strict, _ = _hybrid('rule_gated', 1, rule, depth)
    out = strict.process(None, None, 0.0, vision.X_FAR)
    assert np.array_equal(out.line, _vertical(100))


def test_hybrid_registration_and_parameter_defaults(monkeypatch):
    made = {}

    class FakeRule:
        def __init__(self, node):
            made['rule'] = self

    class FakeDepth:
        def __init__(self, node):
            made['depth'] = self

    monkeypatch.setattr(hybrid_mod, 'RulePerception', FakeRule)
    monkeypatch.setattr(hybrid_mod, 'DepthPerception', FakeDepth)
    sensor = make_perception('hybrid', FakeNode())
    assert isinstance(sensor, HybridPerception)
    assert sensor.mode == 'rule_gated' and sensor.gate_px == 2
    assert set(made) == {'rule', 'depth'}

    sensor = HybridPerception(FakeNode({'hybrid.mode': 'and', 'hybrid.gate_px': 1}))
    assert sensor.mode == 'and' and sensor.gate_px == 1
    with pytest.raises(ValueError):
        HybridPerception(FakeNode({'hybrid.mode': 'nope'}))
