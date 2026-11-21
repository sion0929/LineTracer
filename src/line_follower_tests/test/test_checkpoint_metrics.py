"""Dev checker thresholds must reject missing data and wrong drive directions."""
import importlib.util
from pathlib import Path

import pytest

PATH = Path(__file__).resolve().parents[3] / 'tools' / 'checkpoint_metrics.py'
spec = importlib.util.spec_from_file_location('checkpoint_metrics', PATH)
metrics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(metrics)


def test_start_frame_and_wrap():
    import math
    forward, lateral, yaw = metrics.pose_delta((1, 2, math.pi / 2), (1, 2.08, math.pi / 2))
    assert forward == pytest.approx(.08)
    assert lateral == pytest.approx(0, abs=1e-9)
    assert yaw == 0
    assert metrics.pose_delta((0, 0, 3.1), (0, 0, -3.1))[2] > 0


@pytest.mark.parametrize('v,w,delta,ok', [
    (.06, 0, (.06, 0, 0), True),
    (-.06, 0, (-.06, 0, 0), True),
    (-.06, 0, (.06, 0, 0), False),
    (.04, .5, (.04, .01, .5), True),
    (.04, -.5, (.04, -.01, -.5), True),
    (.04, -.5, (.04, .01, .5), False),
    (0, .6, (0, 0, .6), True),
    (0, -.6, (0, 0, -.6), True),
    (0, .6, (.05, 0, .6), False),
    (.06, 0, (0, 0, 0), False),
])
def test_motion_direction(v, w, delta, ok):
    assert metrics.motion_ok(v, w, 1.0, delta) is ok


def test_rate_needs_samples():
    assert metrics.sample_rate([]) == 0
    assert metrics.sample_rate([1]) == 0
    assert metrics.sample_rate([1, 1.1, 1.2]) == pytest.approx(10)


@pytest.mark.parametrize('delta', [(.06, .08, 0), (.06, 0, .4)])
def test_straight_rejects_large_lateral_or_yaw(delta):
    assert metrics.motion_ok(.06, 0, 1., delta) is False
