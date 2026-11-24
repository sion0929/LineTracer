"""판단 (planning: speed plan) + 제어 (control: pure pursuit, command limits): `control/rule.py`.

RuleController.compute(rp, dt) is driven with a fake node (parameter dict + the
state attributes listed in control/base.py), so no ROS graph is needed.

판단 part (front of compute): v_max / speed_profile windows, curvature limit
sqrt(a_lat * r), braking for an open path end and for the finish T, the hill limit.
제어 part (back of compute): acceleration ramp, 1 m/s^2 deceleration limit, pure
pursuit w = 2 v y / d^2, turning in place when the target is > 60 deg off, omega_max.

Pinned facts (measured):
  * v jumps straight to v_min on the first tick (v_min is a floor, not ramped)
  * on a circle w = v / r exactly and v settles at sqrt(a_lat r) within 0.1 %
  * turning in place forces v <= v_min immediately, overriding the 1 m/s^2 decel limit
  * speed_profile windows REPLACE v_max (they may raise it: 0.7 m/s with v_max 0.25)
"""
import math
import re
import types
from pathlib import Path

import numpy as np
import pytest

import synth
from line_follower.control.rule import RuleController

PKG = Path(__file__).resolve().parents[1]
DT = 0.02
# FollowerNode.declare_parameter defaults
DEFAULTS = dict(v_max=0.25, v_min=0.06, a_lat=0.25, a_dec=0.3, a_acc=0.6, v_hill=0.25,
                omega_max=3.0, lookahead_min=0.10, lookahead_gain=0.30, speed_profile=[0.0])


class FakeNode:
    """The slice of FollowerNode that the controller reads."""

    def __init__(self, **params):
        self.params = dict(DEFAULTS, **params)
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.path_end = 'len'
        self.t_point = None
        self.travel = 0.0
        self.pitch_f = 0.0
        self.pose = (0.0, 0.0, 0.0)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.params[name])

    def to_robot(self, X, Y):
        px, py, th = self.pose
        dx, dy = X - px, Y - py
        return dx * math.cos(th) + dy * math.sin(th), -dx * math.sin(th) + dy * math.cos(th)


def run(node, rp, ticks, dt=DT):
    """Call compute like the node does (feeding back v_cmd / w_cmd); return [(v, w)]."""
    ctl = RuleController(node)
    out = []
    for _ in range(ticks):
        v, w = ctl.compute(rp, dt)
        node.v_cmd, node.w_cmd = v, w
        out.append((v, w))
    return out


def straight_path(length=0.8, step=0.02):
    return [(step * i, 0.0) for i in range(int(round(length / step)) + 1)]


def arc_path(r, length=0.8):
    return [tuple(p) for p in synth.arc(r, length, n=int(round(length / 0.02)) + 1)]


def heading_path(deg, n=30):
    a = math.radians(deg)
    return [(0.02 * i * math.cos(a), 0.02 * i * math.sin(a)) for i in range(n)]


# ------------------------------------------------------------------ 제어: ramp, straight
def test_straight_path_ramps_to_v_max_without_turning():
    node = FakeNode()
    cmds = run(node, straight_path(), 40)
    vs = [v for v, _ in cmds]
    assert all(w == 0.0 for _, w in cmds)
    assert vs[0] == pytest.approx(DEFAULTS['v_min'])      # first tick jumps to the v_min floor
    steps = np.diff(vs)
    assert (steps <= DEFAULTS['a_acc'] * DT + 1e-12).all()
    assert (steps >= -1e-12).all()
    assert max(vs) == pytest.approx(DEFAULTS['v_max']) and max(vs) <= DEFAULTS['v_max'] + 1e-12
    assert vs.index(max(vs)) == 16                         # (0.25 - 0.06) / 0.012 -> 16 ticks


def test_deceleration_is_limited_to_1_m_s2():
    node = FakeNode(v_max=0.7)
    node.v_cmd = 0.7
    node.params['v_max'] = 0.25                            # top speed drops suddenly
    vs = [v for v, _ in run(node, straight_path(), 30)]
    assert np.diff([0.7] + vs).min() == pytest.approx(-1.0 * DT)
    assert vs[:5] == pytest.approx([0.68, 0.66, 0.64, 0.62, 0.60])
    assert vs[-1] == pytest.approx(0.25)


# ------------------------------------------------------------------ 판단: curvature limit
@pytest.mark.parametrize('r', [0.3, 0.5, 1.0, -0.5])
def test_curve_speed_is_limited_by_lateral_acceleration(r):
    node = FakeNode(v_max=1.0)
    cmds = run(node, arc_path(r), 200)
    v_curve = math.sqrt(DEFAULTS['a_lat'] * abs(r))
    assert max(v for v, _ in cmds) <= v_curve + 1e-3
    v, w = cmds[-1]
    assert v == pytest.approx(v_curve, rel=0.002)
    assert w == pytest.approx(v / r, rel=0.002)          # pure pursuit on a circle: w = v / r


def test_curve_braking_starts_before_the_curve():
    """A tight curve 0.5 m ahead caps the speed now by what a_dec can brake."""
    straight = straight_path(0.5)
    tail = [(0.5 + x, y) for x, y in arc_path(0.2, 0.3)[1:]]
    node = FakeNode(v_max=1.0)
    v = run(node, straight + tail, 200)[-1][0]
    v_c = math.sqrt(DEFAULTS['a_lat'] * 0.2)

    def brake(s):                       # speed that can still be braked to v_c in s - 10 cm
        return math.sqrt(v_c ** 2 + 2 * DEFAULTS['a_dec'] * (s - 0.10))

    # curvature is measured on >= 5 cm chords, so the curve registers at s = 0.50 .. 0.60 m
    # (measured: 0.563 m/s, i.e. s = 0.544 m)
    assert brake(0.50) <= v <= brake(0.60)
    assert v_c < v < 1.0


# ------------------------------------------------------------------ 제어: turn in place, clamp
@pytest.mark.parametrize('deg', [65.0, 90.0, -70.0, 120.0])
def test_target_far_off_turns_in_place(deg):
    node = FakeNode(v_max=0.7, omega_max=8.0)
    node.v_cmd = 0.5
    v, w = RuleController(node).compute(heading_path(deg), DT)
    assert v == pytest.approx(DEFAULTS['v_min'])          # 0.5 -> 0.06 in one tick
    assert w == pytest.approx(2.5 * math.radians(deg))
    node = FakeNode(v_max=0.7)                              # default omega_max 3.0
    node.v_cmd = 0.5
    _, w = RuleController(node).compute(heading_path(deg), DT)
    assert w == pytest.approx(math.copysign(min(3.0, 2.5 * abs(math.radians(deg))), deg))


def test_target_less_than_60_deg_off_keeps_driving():
    node = FakeNode(v_max=0.7, omega_max=8.0)
    node.v_cmd = 0.5
    rp = heading_path(55.0)
    v, w = RuleController(node).compute(rp, DT)
    assert v == pytest.approx(0.5 + DEFAULTS['a_acc'] * DT)
    # pure pursuit to the first point beyond the lookahead 0.10 + 0.30 v
    ld = DEFAULTS['lookahead_min'] + DEFAULTS['lookahead_gain'] * v
    tx, ty = next(p for p in rp if math.hypot(*p) >= ld)
    assert w == pytest.approx(2.0 * v * ty / (tx * tx + ty * ty))


@pytest.mark.parametrize('omega_max', [1.5, 3.0])
def test_angular_speed_never_exceeds_omega_max(omega_max):
    rng = np.random.default_rng(7)
    paths = [arc_path(r) for r in (0.1, -0.1, 0.15, -0.2, 0.3)]
    paths += [heading_path(d) for d in (45, -50, 59, -61, 89, 150, -179)]
    for _ in range(20):
        a = rng.uniform(-math.pi, math.pi)
        paths.append([(0.02 * i * math.cos(a) + rng.normal(0, 0.005), 0.02 * i * math.sin(a))
                      for i in range(25)])
    worst = 0.0
    for rp in paths:
        node = FakeNode(v_max=0.7, omega_max=omega_max)
        node.v_cmd = float(rng.uniform(0.0, 0.7))
        for _, w in run(node, rp, 10):
            worst = max(worst, abs(w))
    assert worst <= omega_max + 1e-12
    assert worst == pytest.approx(omega_max)              # the clamp is actually reached


# ------------------------------------------------------------------ 판단: hill
@pytest.mark.parametrize('pitch_deg', [3.1, 4.0, -4.0])
def test_hill_limits_speed_to_v_hill(pitch_deg):
    node = FakeNode(v_max=0.7, v_hill=0.3)
    node.pitch_f = math.radians(pitch_deg)
    vs = [v for v, _ in run(node, straight_path(), 100)]
    assert max(vs) <= 0.3 + 1e-12 and vs[-1] == pytest.approx(0.3)


def test_below_3_deg_is_not_a_hill():
    node = FakeNode(v_max=0.7, v_hill=0.3)
    node.pitch_f = math.radians(2.9)
    assert run(node, straight_path(), 100)[-1][0] == pytest.approx(0.7)


def test_reaching_the_hill_fast_brakes_at_1_m_s2():
    node = FakeNode(v_max=0.7, v_hill=0.3)
    node.v_cmd = 0.6
    node.pitch_f = math.radians(4.0)
    vs = [v for v, _ in run(node, straight_path(), 20)]
    assert vs[:3] == pytest.approx([0.58, 0.56, 0.54])
    assert vs[13] > 0.3 + 1e-6 and vs[14] == pytest.approx(0.3)     # 0.3 s to shed 0.3 m/s


# ------------------------------------------------------------------ 판단: open end / finish T
@pytest.mark.parametrize('length', [0.1, 0.2, 0.3, 0.5, 0.8])
def test_open_path_end_limits_speed_to_what_can_be_braked(length):
    rp = straight_path(length)
    node = FakeNode(v_max=1.0)
    node.path_end = 'open'
    vs = [v for v, _ in run(node, rp, 200)]
    v_min, a_dec = DEFAULTS['v_min'], DEFAULTS['a_dec']
    bound = math.sqrt(v_min ** 2 + 2 * a_dec * max(0.0, length - 0.10))
    assert max(vs) <= bound + 1e-9
    assert vs[-1] >= 0.85 * bound                          # chords >= 5 cm: s <= length
    # from that speed the robot gets down to v_min before (end - 10 cm)
    assert (vs[-1] ** 2 - v_min ** 2) / (2 * a_dec) <= max(0.0, length - 0.10) + 1e-9
    node = FakeNode(v_max=1.0)
    node.path_end = 'len'                                  # same path, but not the end of the line
    assert run(node, rp, 200)[-1][0] == pytest.approx(1.0)


def test_approaching_the_finish_t_slows_down():
    a_dec = DEFAULTS['a_dec']
    speeds = []
    for d in (0.5, 0.2, 0.1, 0.05, 0.02, 0.01):
        node = FakeNode(v_max=1.0)
        node.path_end = 't'
        node.t_point = (d, 0.0)
        v = run(node, straight_path(), 200)[-1][0]
        assert v == pytest.approx(math.sqrt(2 * a_dec * max(0.0, d - 0.01)) + 0.04, abs=1e-6)
        speeds.append(v)
    assert speeds == sorted(speeds, reverse=True)
    assert speeds[-1] == pytest.approx(0.04) and speeds[-1] < DEFAULTS['v_min']


def test_t_point_uses_the_node_pose():
    node = FakeNode(v_max=1.0)
    node.pose = (2.0, 1.0, math.pi / 2)
    node.t_point = (2.0, 1.2)                              # 20 cm straight ahead of that pose
    v = run(node, straight_path(), 200)[-1][0]
    assert v == pytest.approx(math.sqrt(2 * DEFAULTS['a_dec'] * 0.19) + 0.04, abs=1e-6)


# ------------------------------------------------------------------ 판단: speed_profile
def _settled(profile, travel, v_max=0.25):
    node = FakeNode(v_max=v_max, speed_profile=profile)
    node.travel = travel
    return run(node, straight_path(), 200)[-1][0]


def test_speed_profile_windows():
    prof = [1.0, 2.0, 0.5]
    assert _settled(prof, 1.5, v_max=0.25) == pytest.approx(0.5)    # window raises v_max
    assert _settled(prof, 1.5, v_max=0.7) == pytest.approx(0.5)     # ... or lowers it
    assert _settled(prof, 0.5) == pytest.approx(0.25)               # before the window
    assert _settled(prof, 2.0) == pytest.approx(0.25)               # s_to is exclusive
    # a slower window ahead is braked for: sqrt(v_w^2 + 2 a_dec (s_from - travel))
    slow = [1.0, 2.0, 0.1]
    for travel in (0.5, 0.9):
        expect = math.sqrt(0.1 ** 2 + 2 * DEFAULTS['a_dec'] * (1.0 - travel))
        assert _settled(slow, travel, v_max=0.7) == pytest.approx(expect, rel=1e-6)
    assert _settled([1.0, 2.0, 0.01], 1.5) == pytest.approx(DEFAULTS['v_min'])   # v_min floor


def test_default_profile_is_a_no_op():
    for travel in (0.0, 5.0, 13.0):
        for v_max in (0.25, 0.7):
            assert _settled([0.0], travel, v_max) == pytest.approx(_settled([], travel, v_max))
            assert _settled([0.0], travel, v_max) == pytest.approx(v_max)


@pytest.mark.parametrize('config', ['rule.yaml', 'follower.yaml'])
def test_shipped_profile_is_tuned_to_the_official_track(config):
    """The shipped speed_profile keys on distance driven, i.e. on the official line_track
    layout. On any other track it still raises the top speed to 0.7 m/s between 7.2 and
    10.1 m and after 14 m (v_max is 0.25), wherever the robot is."""
    text = (PKG / 'config' / config).read_text()
    found = re.search(r'^\s*speed_profile:\s*\[([^\]]*)\]', text, re.M)
    assert found, 'speed_profile not found'
    prof = [float(v) for v in found.group(1).split(',')]
    assert prof == [7.2, 10.1, 0.7, 12.8, 14.0, 0.45, 14.0, 30.0, 0.7]
    assert _settled(prof, 5.0) == pytest.approx(0.25)
    assert _settled(prof, 8.0) == pytest.approx(0.7)
    assert _settled(prof, 12.0) == pytest.approx(0.25)
    assert _settled(prof, 13.0) == pytest.approx(0.45)
    assert _settled(prof, 20.0) == pytest.approx(0.7)
