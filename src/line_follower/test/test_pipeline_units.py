"""Unit tests for the perception / planning / control split (no ROS node, no Gazebo)."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from line_follower import line_map, vision
from line_follower.line_map import C0, RES, LineMap
from line_follower.planning import make_planner, path_len
from line_follower.vision import thin_across


def test_ground_to_image_round_trip():
    # a ground point straight ahead lands on the image centre column, below the horizon
    u, v = vision.ground_to_image(0.6, 0.0)
    assert abs(u - vision.CX) < 1e-6
    assert v > vision.CY
    # farther ground is closer to the horizon
    assert vision.ground_to_image(1.2, 0.0)[1] < v


def test_far_band_is_a_thin_horizon_strip():
    v_near = vision.ground_to_image(0.85, 0.0)[1]
    v_far = vision.ground_to_image(1.2, 0.0)[1]
    assert 15 < v_near - v_far < 40          # 0.85-1.2 m is only ~20-25 rows of 480


def test_far_band_mask_threshold_finds_a_dark_line():
    gray = np.full((vision.H, vision.W), 200, np.uint8)
    # a dark horizontal stripe across the image ~1.0 m ahead
    v = int(round(vision.ground_to_image(1.0, 0.0)[1]))
    gray[v - 1:v + 2] = 20
    line, valid = vision.far_band_mask(gray, 0.0, 0.85, min(1.2, vision.X_FAR))
    if vision.X_FAR <= 0.85:
        assert not line.any()               # BEV ends before the band: nothing to report
    else:
        assert line.any() and valid.any()


def test_thin_across_keeps_long_runs_and_thins_short_ones():
    m = np.zeros((60, 5), bool)
    m[10:18, 0] = True                     # short across-run (8 cells) -> centre only
    m[5:55, 1] = True                      # long run (a line running away) -> kept
    out = thin_across(m, max_run=16, keep=1)
    assert out[:, 0].sum() == 3
    assert out[:, 1].sum() == 50


def test_restored_layer_is_read_like_seen_line():
    lm = LineMap()
    X, Y = 0.40, 0.10
    assert not lm.hit(X, Y)
    lm.restored = np.zeros_like(lm.grid)
    i, j = int(round(Y / RES)) + C0, int(round(X / RES)) + C0
    lm.restored[i, j] = 255
    assert lm.hit(X, Y)
    snapped = lm.snap(X + 0.01, Y, 0.03)
    assert snapped is not None and math.hypot(snapped[0] - X, snapped[1] - Y) < 0.02


def test_freeze_blocks_rewrites():
    lm = LineMap()
    lm.freeze(0.0, 0.0, 0.2)
    assert lm.frozen[C0, C0] == 255
    assert lm.frozen[C0, C0 + int(0.3 / RES)] == 0


def test_planner_plugin_and_path_len():
    node = SimpleNamespace()
    planner = make_planner('rule', node)
    assert planner.node is node
    assert path_len([(0, 0), (0.3, 0.4), (0.3, 1.4)]) == pytest.approx(1.5)
    with pytest.raises(ValueError):
        make_planner('nope', node)


def _controller(kp=0.0, ki=0.0, kd=0.0):
    from line_follower.control.rule import RuleController
    params = dict(v_max=0.25, v_min=0.06, a_lat=0.25, a_dec=0.3, a_acc=0.6, omega_max=3.0,
                  speed_profile=[0.0], v_hill=0.25, lookahead_min=0.10, lookahead_gain=0.30,
                  pid_kp=kp, pid_ki=ki, pid_kd=kd)
    node = SimpleNamespace(travel=1.0, pitch_f=0.0, t_point=None, path_end='len', v_cmd=0.2, w_cmd=0.0,
                           get_parameter=lambda k: SimpleNamespace(value=params[k]))
    return RuleController(node)


def test_pid_zero_gains_is_plain_pure_pursuit():
    rp = [(0.02 * k, 0.03 + 0.001 * k) for k in range(1, 40)]
    v0, w0 = _controller().compute(rp, 0.02)
    v1, w1 = _controller(kp=5.0).compute(rp, 0.02)
    assert v1 == pytest.approx(v0)
    assert w1 > w0                         # path left of the axle: the P term steers further left


def test_patch_grows_with_bev_range():
    assert line_map.PATCH >= 2 * math.hypot(vision.X_FAR, vision.Y_HALF) / RES


def test_map_restoration_extends_only_past_the_seen_area():
    from line_follower.restore import Restorer
    r = object.__new__(Restorer)                 # no ONNX: a stub net predicting the line y = 0
    r.calls = 0

    def route(lm, pose):
        prob = np.zeros((128, 128), np.float32)
        prob[:, 63:65] = 0.999                   # robot-centric patch column 64 = y 0
        return prob
    r.route = route
    lm = LineMap()
    for x in np.arange(-0.3, 0.8, RES):
        for y in np.arange(-0.3, 0.3, RES):
            lm.seen[int(round(y / RES)) + C0, int(round(x / RES)) + C0] = 255
        lm.grid[C0, int(round(x / RES)) + C0] = 255
    lm.write_restored(r, (0.0, 0.0, 0.0), 0.6, 0.4)
    ys, xs = np.nonzero(lm.restored)
    X = (xs - C0) * RES
    assert len(xs) > 0
    assert X.min() > 0.75 and X.max() < 1.25          # past the seen area, at most ~0.4 m
    assert not ((lm.restored > 0) & (lm.seen > 0)).any()


def test_stages_keep_shared_state_on_the_node():
    """Perception / planning state lives on the node (shared with control and dev tools).
    Reading it from the stage object silently disables features (e.g. the ramp deck cap)."""
    import inspect
    import re
    from line_follower.perception import stage
    from line_follower.planning import rule
    own = {'t_restore'}                                   # stage-private timers
    for mod in (stage, rule):
        src = inspect.getsource(mod)
        for name in re.findall(r"(?:get|has)attr\(self, '(\w+)'", src):
            assert name in own, f"{mod.__name__} reads '{name}' from self, not from node"
        assert 'correct(self,' not in src


def test_confidence_is_handed_with_the_map():
    lm = LineMap()
    bev = np.zeros((vision.BH, vision.BW), np.uint8)
    valid = np.full_like(bev, 255)
    col = vision.BW // 2
    bev[:, col - 2:col + 3] = 255                       # a line straight ahead
    lm.update(bev, valid, (0.0, 0.0, 0.0))
    near = lm.confidence(0.4, 0.0)
    far = lm.confidence(vision.X_FAR - 0.03, 0.0)
    assert near > 0.9 and 0 < far <= near
    assert lm.confidence(0.4, 0.3) == 0.0                # no line there
    lm.restored = np.zeros_like(lm.grid)
    lm.restored[C0, C0 + int(round(2.0 / RES))] = 255
    assert 0 < lm.confidence(2.0, 0.0) < far


def test_trt_resolve_prefers_sibling_engine(tmp_path, monkeypatch):
    from line_follower.trt_runner import resolve_model
    onnx = tmp_path / 'V4-NearB320-Isaac.onnx'
    onnx.write_bytes(b'x')
    assert resolve_model(str(onnx)) == (onnx, False)          # PC: no engine -> ONNX path
    engine = tmp_path / 'V4-NearB320-Isaac.engine'
    engine.write_bytes(b'x')
    assert resolve_model(str(onnx)) == (engine, True)         # Jetson: engine next to it wins
    monkeypatch.setenv('LF_NO_TRT', '1')
    assert resolve_model(str(onnx)) == (onnx, False)
    assert resolve_model(str(engine)) == (engine, True)
