"""CPU tests for depth projection with synthetic network outputs."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from line_follower import vision
from line_follower.perception.depth import DepthPerception, H_IN, REST_PITCH, W_IN


@pytest.fixture
def projection():
    # Bypass model loading while exercising the real footprint and projection.
    sensor = DepthPerception.__new__(DepthPerception)
    sensor.node = SimpleNamespace(imu=[])
    sensor.max_depth, sensor.margin = 1.4, 0.0
    sensor.use_pitch = False
    sensor.splat = 'dilate'
    sensor.ru = (np.arange(W_IN) + 0.5 - W_IN / 2) / (vision.FX * W_IN / vision.W)
    sensor.rv = (np.arange(H_IN) + 0.5 - H_IN / 2) / (vision.FX * H_IN / vision.H)
    sensor.kernel = np.ones((3, 3), np.uint8)
    logits = np.zeros((2, H_IN, W_IN), np.float32)
    depth = np.full((H_IN, W_IN), 0.5, np.float32)
    sensor.infer = lambda _: (logits, depth)
    gray = np.zeros((vision.H, vision.W), np.uint8)
    return sensor, logits, depth, gray


def test_splat_preserves_legacy_dilation(projection):
    sensor, logits, _, gray = projection
    logits[1, 170, 160] = 1
    output = sensor.process(None, gray, REST_PITCH, vision.X_FAR)
    assert np.count_nonzero(output.line) == 25
    assert output.line.dtype == output.valid.dtype == np.uint8
    assert output.obstacle is None


def test_centre_keeps_separate_runs_and_ignores_invalid_depth(projection):
    sensor, logits, depth, gray = projection
    sensor.splat = 'centre'
    logits[1, 170, 142:151] = 1
    logits[1, 170, 174:179] = 1
    logits[1, 170, 190:195] = 1
    depth[170, 190:195] = np.nan
    output = sensor.process(None, gray, REST_PITCH, vision.X_FAR)
    expected = np.rint((vision.Y_HALF + sensor.ru[[146, 176]] * 0.5) / vision.RES)
    assert np.array_equal(np.nonzero(output.line)[1], expected)


@pytest.mark.parametrize('pitch_deg', [-12, 12])
def test_opt_in_levels_depth_using_raw_pitch_when_stage_clamps(projection, pitch_deg):
    sensor, logits, _, gray = projection
    sensor.splat = 'close'
    sensor.use_pitch = True
    logits[1, 170, 160] = 1
    pitch = math.radians(pitch_deg)
    sensor.node.imu = [(1.0, pitch)]
    # Camera mount is level, 76 mm forward and 93 mm above the Isaac base origin.
    expected_x = (0.076 + 0.5) * math.cos(pitch) - (
        0.093 - 0.5 * sensor.rv[170]) * math.sin(pitch)
    output = sensor.process(None, gray, REST_PITCH, vision.X_FAR)
    row, _ = np.nonzero(output.line)
    assert row.tolist() == [round((vision.X_FAR - expected_x) / vision.RES)]


def test_opt_in_uses_supplied_pitch_without_imu_and_keeps_range_gate(projection):
    sensor, logits, _, gray = projection
    sensor.use_pitch = True
    sensor.splat = 'close'
    logits[1, 170, 160] = 1
    pitch = math.radians(-2.5)
    expected_x = (0.076 + 0.5) * math.cos(pitch) - (
        0.093 - 0.5 * sensor.rv[170]) * math.sin(pitch)
    output = sensor.process(None, gray, pitch, vision.X_FAR)
    assert np.nonzero(output.line)[0].tolist() == [
        round((vision.X_FAR - expected_x) / vision.RES)]
    assert not sensor.process(None, gray, pitch, 0.55).line.any()
