"""판단 (situation recognition): GT labels, runtime features, the shipped classifier and the
playbook finish judge (control/playbook.py).

* labels: experiments/situation/label.py on small hand-made maps (skipped when the dev tree
  is not next to the package)
* features: line_follower.situation.features on LineMap grids painted with synth helpers
* model: config/situation_model.yaml loads, gives probabilities, runs well under 2 ms, and keeps
  a per-class accuracy floor on real Gazebo samples (test/data/situation_fixture.npz, held-out maps)
* finish: PlaybookController.confirm_finish accepts a real T (after its confirmation frames)
  and rejects a crossing bar, an unseen-beyond bar, a stub and the start T - with travel 0,
  i.e. without any driven-distance gate
"""
import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

import synth
from line_follower import line_map
from line_follower.line_map import LineMap
from line_follower.situation import INDEX, SITUATIONS
from line_follower.situation import features as FE

PKG = Path(__file__).resolve().parents[1]
WS = PKG.parents[1]
DEV = WS / 'experiments' / 'situation'
FIXTURE = PKG / 'test' / 'data' / 'situation_fixture.npz'
paint = synth.to_grid_painter(line_map)


def painted(*polylines):
    m = LineMap()
    paint(m.grid, [np.asarray(p, float) for p in polylines])
    return m


def line(*pts, step=0.02):
    pts = np.asarray(pts, float)
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        k = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * i / k for i in range(1, k + 1)]
    return np.array(out)


# ------------------------------------------------------------------ labels (dev tool)
@pytest.fixture(scope='module')
def label():
    if not (DEV / 'label.py').is_file():
        pytest.skip('experiments/situation not present')
    sys.path.insert(0, str(DEV))
    import label as mod
    return mod


def meta(centre, markings=(), **extra):
    c = [[x, y, 0.0] for x, y in centre]
    return dict(centerline=c, markings=[[[x, y, 0.0] for x, y in m] for m in markings], **extra)


@pytest.fixture(scope='module')
def course(label):
    """(0,0) -> (2,0) square corner -> (2,1.5) arc r 0.3 -> (0.5,1.5) finish.
    Start bar at x=0, finish bar at x=0.5, cross bar at x=1.0, stub at x=1.6 (left)."""
    a = line((0, 0), (2, 0), (2, 1.2))
    arc = [(2 - 0.3 + 0.3 * math.cos(t), 1.2 + 0.3 * math.sin(t)) for t in np.linspace(0, math.pi / 2, 24)[1:]]
    b = line((1.7, 1.5), (0.5, 1.5))[1:]
    centre = np.vstack((a, arc, b))
    marks = [[(0, -0.1), (0, 0.1)], [(0.5, 1.4), (0.5, 1.6)], [(1.0, -0.2), (1.0, 0.2)], [(1.6, 0.0), (1.6, 0.25)]]
    return label.Track(meta(centre, marks))


@pytest.mark.parametrize('pose, expected', [
    ((0.10, 0.0, 0.0), 'STRAIGHT'),          # cross bar 0.9 m ahead: still outside the window
    ((0.40, 0.0, 0.0), 'CROSSING'),
    ((1.20, 0.0, 0.0), 'STUB'),              # stub (1.6) before the corner (2.0)
    ((1.75, 0.0, 0.0), 'RIGHT_ANGLE'),
    ((2.0, 0.2, math.pi / 2), 'STRAIGHT'),
    ((2.0, 0.8, math.pi / 2), 'ARC'),
    ((1.20, 1.5, math.pi), 'FINISH_T'),
    ((1.00, 0.6, 0.0), 'OFF'),
    ((0.60, 0.0, math.pi), 'START_T'),       # driving back, facing the start bar
    ((0.60, 0.0, -math.pi / 2), 'OFF'),
])
def test_window_labels(course, pose, expected):
    assert course.label(pose) == expected


def test_t_candidate_labels(course):
    # robot 0.7 m before the finish: the window says FINISH_T, but a T candidate at the
    # cross bar (or anywhere else) is not the finish
    assert course.label((1.2, 1.5, math.pi), cand=(0.5, 1.5)) == 'FINISH_T'
    assert course.label((1.2, 1.5, math.pi), cand=(0.9, 1.5)) == 'STRAIGHT'
    assert course.label((0.4, 0.0, 0.0), cand=(1.0, 0.0)) == 'CROSSING'
    assert course.candidate_kind((1.6, 0.0)) == 'STUB'
    assert course.candidate_kind((0.0, 0.0)) == 'START_T'


def test_gap_hill_and_loop_labels(label):
    flat = label.Track(meta(line((0, 0), (4, 0)), gaps_arc_m=[[1.0, 1.1]],
                            ramps=[{'x': [2.5, 2.9, 3.3, 3.7], 'y': [-0.3, 0.3], 'height_m': 0.05}]))
    assert flat.label((0.5, 0.0, 0.0)) == 'GAP'
    assert flat.label((2.2, 0.0, 0.0)) == 'HILL'
    assert flat.label((3.0, 0.0, 0.0)) == 'HILL'
    # straight to (1.1, 0), 270 deg left circle r 0.1, then down through (1.0, 0): a small loop
    circ = [(1.1 + 0.1 * math.cos(t), 0.1 + 0.1 * math.sin(t))
            for t in np.linspace(-math.pi / 2, math.pi, 40)[1:]]
    loop = label.Track(meta(np.vstack((line((0, 0), (1.1, 0)), circ, line((1.0, 0.1), (1.0, -2.0))[1:]))))
    assert loop.loops, 'self-crossing 0.7 m apart must be a small loop'
    assert loop.label((0.5, 0.0, 0.0)) == 'LOOP_CORNER'
    assert loop.label((1.0, -0.4, -math.pi / 2)) == 'STRAIGHT'


# ------------------------------------------------------------------ features
def feats(m, pose, path, end, start=(-3.0, 0.0)):
    return FE.extract(m.grid, pose, path, end, 0.0, start)


def test_patch_is_robot_frame():
    m = painted(line((0.0, 0.10), (1.0, 0.10)))               # line 10 cm LEFT of the robot
    patch = FE.robot_patch(m.grid, (0.0, 0.0, 0.0))
    assert patch.shape == (FE.PH, FE.PW)
    col_left = int(round((FE.Y_HALF - 0.10) / FE.PX))
    row = int(round((FE.X_MAX - 0.5) / FE.PX))
    assert patch[row, col_left] and not patch[row, FE.PW - 1 - col_left]
    # same world line, robot turned 90 deg left at (0.1, -0.5): the line is now straight ahead
    patch = FE.robot_patch(m.grid, (0.1, -0.5, math.pi / 2))
    assert patch[int(round((FE.X_MAX - 0.6) / FE.PX)), int(round(FE.Y_HALF / FE.PX))]


def test_straight_line_features():
    m = painted(line((0.0, 0.0), (1.2, 0.0)))
    path, end = m.trace(0.0, 0.0, 0.0)
    f = feats(m, (0.0, 0.0, 0.0), path, end)
    assert f.shape == (len(FE.FEATURE_NAMES),) and f.dtype == np.float32 and np.isfinite(f).all()
    assert end == 'len' and f[FE.F['path_lenlim']] == 1
    assert f[FE.F['path_turn']] < 0.05 and f[FE.F['path_sharp']] < 0.1
    assert f[FE.F['junc_left']] == f[FE.F['junc_right']] == 0
    assert f[FE.F['occ_2c']] > 0.5 and f[FE.F['occ_2l']] == f[FE.F['occ_2r']] == 0


def test_t_bar_features():
    m = painted(line((0.0, 0.0), (0.56, 0.0)), line((0.57, -0.10), (0.57, 0.10)))
    path, end = m.trace(0.0, 0.0, 0.0)
    assert end == 't'
    f = feats(m, (0.0, 0.0, 0.0), path, end)
    assert f[FE.F['path_t']] == 1
    assert 0.07 <= f[FE.F['end_arm_l']] <= 0.13 and 0.07 <= f[FE.F['end_arm_r']] <= 0.13
    assert f[FE.F['end_beyond_hit']] == 0
    assert f[FE.F['end_beyond_seen']] >= 0.15            # the camera sees ~0.25 m past the bar


def test_crossing_features_see_the_line_beyond():
    m = painted(line((0.0, 0.0), (1.2, 0.0)), line((0.6, -0.2), (0.6, 0.2)))
    path = [tuple(p) for p in line((0.0, 0.0), (0.58, 0.0))]
    f = feats(m, (0.0, 0.0, 0.0), path, 't')
    assert f[FE.F['end_beyond_hit']] > 0.2           # 1 of 3 lateral probes per step: <= 1/3
    assert f[FE.F['end_arm_l']] > 0.15 and f[FE.F['end_arm_r']] > 0.15


@pytest.mark.parametrize('side', [+1, -1])
def test_side_branch_shows_on_its_side(side):
    m = painted(line((0.0, 0.0), (1.2, 0.0)), line((0.5, 0.0), (0.5, side * 0.25)))
    path, end = m.trace(0.0, 0.0, 0.0)
    f = feats(m, (0.0, 0.0, 0.0), path, end)
    near, far = ('junc_left', 'junc_right') if side > 0 else ('junc_right', 'junc_left')
    assert f[FE.F[near]] >= 1 and f[FE.F[far]] == 0


def test_start_context():
    m = painted(line((0.0, 0.0), (1.0, 0.0)))
    f = FE.extract(m.grid, (0.6, 0.0, math.pi), [], 'open', 0.0, (0.0, 0.0))
    assert f[FE.F['ctx_near_start']] == pytest.approx(0.6) and f[FE.F['ctx_start_facing']] == pytest.approx(1.0)
    f = FE.extract(m.grid, (0.6, 0.0, 0.0), [], 'open', math.radians(5), (0.0, 0.0))
    assert f[FE.F['ctx_start_facing']] == pytest.approx(-1.0) and f[FE.F['ctx_pitch']] == pytest.approx(5.0)


# ------------------------------------------------------------------ the shipped model
@pytest.fixture(scope='module')
def model():
    from line_follower.situation.model import SituationModel
    return SituationModel()


def test_model_loads_and_predicts(model):
    m = painted(line((0.0, 0.0), (1.2, 0.0)))
    path, end = m.trace(0.0, 0.0, 0.0)
    p = model.proba(feats(m, (0.0, 0.0, 0.0), path, end))
    assert p.shape == (len(SITUATIONS),) and np.all(p >= 0) and p.sum() == pytest.approx(1.0)
    assert SITUATIONS[int(p.argmax())] in ('STRAIGHT', 'HILL', 'ARC')


def test_inference_is_fast(model):
    m = painted(line((0.0, 0.0), (0.56, 0.0)), line((0.57, -0.10), (0.57, 0.10)))
    path, end = m.trace(0.0, 0.0, 0.0)
    f = feats(m, (0.0, 0.0, 0.0), path, end)
    t = time.perf_counter()
    for _ in range(200):
        model.proba(f)
    per_call = (time.perf_counter() - t) / 200
    assert per_call < 0.002
    t = time.perf_counter()
    for _ in range(50):
        model.proba(feats(m, (0.0, 0.0, 0.0), path, end))
    assert (time.perf_counter() - t) / 50 < 0.004          # features + model (measured ~0.6 ms)


def test_model_file_is_small():
    assert (PKG / 'config' / 'situation_model.yaml').stat().st_size < 1_000_000


# per-class accuracy floors on real Gazebo samples of held-out maps (gz_val); measured values
# are in experiments/results/situation/model/metrics.md
# (fixture of 25 per class, half of them T candidates where possible; measured 2026-11-25:
#  STRAIGHT 0.24 ARC 0.84 RIGHT_ANGLE 0.68 CROSSING 0.80 LOOP_CORNER 0.68 STUB 0.60 GAP 0.44
#  HILL 0.84 FINISH_T 0.92 OFF 0.52)
FLOORS = {'STRAIGHT': 0.15, 'ARC': 0.7, 'RIGHT_ANGLE': 0.55, 'CROSSING': 0.65, 'LOOP_CORNER': 0.55,
          'STUB': 0.45, 'GAP': 0.3, 'HILL': 0.7, 'FINISH_T': 0.8, 'OFF': 0.4}


def fixture_rows():
    z = np.load(FIXTURE)
    ends = {0: 'open', 1: 't', 2: 'len'}
    for i in range(len(z['y'])):
        patch = np.unpackbits(z['patch'][i])[:FE.PH * FE.PW].reshape(FE.PH, FE.PW).astype(bool)
        n = int(z['n'][i])
        f = FE.extract_local(patch, z['rp'][i][:n] if n >= 2 else None, ends.get(int(z['end'][i]), 'none'),
                             float(z['pitch'][i]), z['start'][i], float(z['x_far'][i]))
        yield f, int(z['y'][i]), int(z['tcand'][i])


def test_fixture_is_small_and_labelled():
    assert FIXTURE.stat().st_size < 300_000
    z = np.load(FIXTURE)
    assert tuple(z['classes']) == SITUATIONS
    assert len(set(z['y'].tolist())) >= 6


def test_accuracy_floor_on_real_gazebo_patches(model):
    from line_follower.control.playbook import DEFAULTS
    fin = DEFAULTS['finish']
    hit, n = np.zeros(len(SITUATIONS)), np.zeros(len(SITUATIONS))
    false_finish = 0
    for f, y, tcand in fixture_rows():
        p = model.proba(f)
        pred = int(p.argmax())
        n[y] += 1
        hit[y] += pred == y
        # one confirming frame of PlaybookController.confirm_finish on a candidate that is not the finish
        judged = (p[INDEX['FINISH_T']] >= fin['p_min'] and f[FE.F['end_beyond_seen']] >= fin['beyond_seen_min']
                  and f[FE.F['end_beyond_hit']] <= fin['beyond_hit_max'])
        false_finish += bool(tcand and judged and y != INDEX['FINISH_T'])
    for name, floor in FLOORS.items():
        k = INDEX[name]
        if n[k]:
            assert hit[k] / n[k] >= floor, f'{name}: {hit[k]:.0f}/{n[k]:.0f}'
    assert false_finish == 0


# ------------------------------------------------------------------ playbook finish judge
class FakeNode:
    """The slice of FollowerNode the playbook controller reads."""

    PARAMS = dict(v_max=0.25, v_min=0.06, a_lat=0.25, a_dec=0.3, a_acc=0.6, v_hill=0.25, omega_max=3.0,
                  lookahead_min=0.10, lookahead_gain=0.30, speed_profile=[0.0], finish_min_dist=14.0)

    def __init__(self, grid_map, pose=(0.0, 0.0, 0.0)):
        self.map, self.pose = grid_map, pose
        self.params = dict(self.PARAMS)
        self.path, self.path_end, self.t_point = [], 'open', None
        self.pitch_f = self.rock = self.v_cmd = self.w_cmd = self.travel = 0.0
        self.logged = []

    def has_parameter(self, name):
        return name in self.params

    def declare_parameter(self, name, value):
        self.params[name] = value

    def get_parameter(self, name):
        return type('P', (), {'value': self.params[name]})()

    def get_clock(self):
        return type('C', (), {'now': lambda s: type('T', (), {'nanoseconds': 0})()})()

    def get_logger(self):
        return type('L', (), {'info': lambda s, msg: self.logged.append(msg)})()

    def to_robot(self, X, Y):
        px, py, th = self.pose
        dx, dy = X - px, Y - py
        return dx * math.cos(th) + dy * math.sin(th), -dx * math.sin(th) + dy * math.cos(th)


def judge(m, path, frames=6, pose=(0.0, 0.0, 0.0), start=(-3.0, 0.0)):
    from line_follower.control.playbook import PlaybookController
    node = FakeNode(m, pose)
    ctl = PlaybookController(node)
    ctl.start = start
    node.path, node.path_end = [tuple(p) for p in path], 't'
    return [ctl.confirm_finish(node.path, None) for _ in range(frames)], ctl


def test_real_t_is_accepted_without_distance_gate():
    m = painted(line((-0.5, 0.0), (0.56, 0.0)), line((0.57, -0.10), (0.57, 0.10)))
    path, end = m.trace(0.0, 0.0, 0.0)
    assert end == 't'
    verdicts, ctl = judge(m, path)
    need = ctl.cfg['finish']['frames']
    assert verdicts[:need - 1] == [False] * (need - 1) and verdicts[need - 1] is True
    assert ctl.last_finish_p >= ctl.cfg['finish']['p_min']


def test_crossing_bar_is_rejected():
    # the bar is seen and the route continues past it (in the map)
    m = painted(line((-0.5, 0.0), (1.2, 0.0)), line((0.6, -0.2), (0.6, 0.2)))
    verdicts, _ = judge(m, line((0.0, 0.0), (0.58, 0.0)))
    assert not any(verdicts)


def test_bar_at_the_edge_of_view_is_not_accepted_yet():
    # nothing beyond the bar has been seen yet: a crossing and a T look the same
    m = painted(line((-0.5, 0.0), (0.80, 0.0)), line((0.81, -0.10), (0.81, 0.10)))
    verdicts, _ = judge(m, line((0.0, 0.0), (0.79, 0.0)))
    assert not any(verdicts)


def test_stub_is_rejected():
    # on the route: the stub's T-shaped root while the route ahead is already mapped
    m = painted(line((-0.5, 0.0), (1.2, 0.0)), line((0.6, 0.0), (0.6, 0.25)))
    verdicts, _ = judge(m, line((0.0, 0.0), (0.58, 0.0)))
    assert not any(verdicts)
    # coming along the stub towards the route: a T whose bar is the long route line
    m = painted(line((-0.5, 0.0), (0.56, 0.0)), line((0.57, -0.8), (0.57, 0.8)))
    path, end = m.trace(0.0, 0.0, 0.0)
    assert end == 't'
    verdicts, _ = judge(m, path)
    assert not any(verdicts)


def test_start_t_is_rejected():
    # driving back towards where the run started: the start T is not the finish
    m = painted(line((-0.5, 0.0), (0.56, 0.0)), line((0.57, -0.10), (0.57, 0.10)))
    path, end = m.trace(0.0, 0.0, 0.0)
    verdicts, _ = judge(m, path, start=(0.57, 0.0))
    assert not any(verdicts)


def test_playbook_is_a_registered_plugin():
    from line_follower.control import make_controller
    from line_follower.control.playbook import PlaybookController
    ctl = make_controller('playbook', FakeNode(LineMap()))
    assert isinstance(ctl, PlaybookController)
    text = (PKG / 'config' / 'playbook_rule.yaml').read_text()
    assert 'controller: playbook' in text
    rule = (PKG / 'config' / 'rule.yaml').read_text()
    strip = [ln for ln in text.splitlines() if 'controller:' not in ln]
    assert strip == [ln for ln in rule.splitlines() if 'controller:' not in ln]


def test_compute_keeps_rule_behaviour_without_actions():
    """With an empty situation table the playbook drives exactly like 'rule'."""
    from line_follower.control.playbook import PlaybookController
    from line_follower.control.rule import RuleController
    m = painted(line((-0.5, 0.0), (1.2, 0.0)))
    path, end = m.trace(0.0, 0.0, 0.0)
    rp = [tuple(p) for p in path]
    a, b = FakeNode(m), FakeNode(m)
    for n in (a, b):
        n.path, n.path_end = rp, end
    ctl = PlaybookController(a)
    ctl.cfg = dict(ctl.cfg, situations={})
    assert ctl.compute(rp, 0.02) == pytest.approx(RuleController(b).compute(rp, 0.02))


def test_situation_row_caps_speed_and_keeps_curvature():
    from line_follower.control.playbook import PlaybookController
    from line_follower.control.rule import RuleController
    arc = synth.arc(0.5, 1.0)                               # gentle left curve, r 0.5 m
    m = painted(arc)
    path, end = m.trace(0.0, 0.0, 0.0)
    rp = [tuple(p) for p in path]
    a, b = FakeNode(m), FakeNode(m)
    for n in (a, b):
        n.path, n.path_end, n.v_cmd = rp, end, 0.25
    ctl = PlaybookController(a)
    ctl.update_situation = lambda: None
    ctl.situation = 'ARC'
    ctl.cfg = dict(ctl.cfg, situations={'ARC': {'v_cap': 0.10, 'omega_max': 1.0}})
    v, w = ctl.compute(rp, 0.02)
    v0, w0 = RuleController(b).compute(rp, 0.02)
    assert v == pytest.approx(0.10) and v0 > 0.10
    assert w / v == pytest.approx(w0 / v0, rel=1e-6)        # same pure-pursuit curvature
    assert abs(w) <= 1.0
