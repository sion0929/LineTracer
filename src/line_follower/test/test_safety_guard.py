"""safety_guard: the single /cmd_vel gate between the follower and the robot.

The follower publishes a command candidate on /line_follower/cmd_raw; the guard is the only
/cmd_vel publisher. Most tests build the node for real (rclpy) but never spin it: callbacks
and tick() are called directly with hand-made messages, ROS time comes from a fake clock
(node.get_clock replaced, as in test_follower_node.py) and wall time from the same fake clock
(the `wall` constructor argument). /cmd_vel and the status topic are captured by replacing
the publishers' publish method.

The last section starts the real guard / follower processes and signals them (SIGINT /
SIGTERM): this is the only way to see that the zero command really leaves the process before
rclpy's context goes away.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import pytest
import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, Imu

import line_follower
from line_follower import safety_guard
from line_follower.safety_guard import SafetyGuard, scale_to_limits

DT = 0.02
PKG = Path(__file__).resolve().parents[1]
CONFIG = PKG / 'config'


# ------------------------------------------------------------------ fixtures / helpers
@pytest.fixture(scope='module')
def ros():
    owns = not rclpy.ok()
    if owns:
        env = os.environ.get('LF_TEST_DOMAIN_ID') or os.environ.get('ROS_DOMAIN_ID')
        rclpy.init(domain_id=int(env) if env else 30 + os.getpid() % 40)
    yield
    if owns:
        rclpy.shutdown()


class FakeClock:
    """ROS time (sim time in Gazebo) and wall time; both advance together unless told."""

    def __init__(self):
        self.ns = 1_000_000_000
        self.wall_s = 100.0

    def now(self):
        return type('T', (), {'nanoseconds': self.ns})()

    def wall(self):
        return self.wall_s

    def advance(self, dt, ros=True, wall=True, rtf=1.0):
        if ros:
            self.ns += int(round(dt * rtf * 1e9))
        if wall:
            self.wall_s += dt

    @property
    def t(self):
        return self.ns * 1e-9


def yaml_params(name):
    data = yaml.safe_load((CONFIG / name).read_text())
    return data['safety_guard']['ros__parameters']


def make_guard(params=None, config=None):
    values = dict(yaml_params(config)) if config else {}
    values.update(params or {})
    values.pop('use_sim_time', None)
    clock = FakeClock()
    g = SafetyGuard(wall=clock.wall,
                    parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
    g.fake_clock = clock
    g.get_clock = lambda: clock
    g.sent = []
    g.cmd_pub.publish = g.sent.append
    g.status = []
    g.status_pub.publish = g.status.append
    g.seq = 0
    return g


@pytest.fixture
def guard(ros):
    made = []

    def build(params=None, config='safety_sim.yaml'):
        params = dict(params or {})
        params.setdefault('start_delay', 0.0)
        g = make_guard(params, config)
        made.append(g)
        return g
    yield build
    for g in made:
        g.destroy_node()


def stamp(msg, t):
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = int(round((t % 1.0) * 1e9)) % 1_000_000_000


def odom_msg(t, x, stamp_t=None):
    m = Odometry()
    stamp(m, t if stamp_t is None else stamp_t)
    m.pose.pose.position.x = float(x)
    m.pose.pose.orientation.w = 1.0
    return m


def imu_msg(t, k, stamp_t=None):
    m = Imu()
    stamp(m, t if stamp_t is None else stamp_t)
    m.orientation.w = 1.0
    m.angular_velocity.z = 1e-4 * (k % 7)          # sensor noise: content always changes
    m.linear_acceleration.z = 9.81 + 1e-4 * (k % 5)
    return m


def image_msg(t, stamp_t=None):
    m = Image()
    stamp(m, t if stamp_t is None else stamp_t)
    return m


def twist(v=0.0, w=0.0):
    tw = Twist()
    tw.linear.x, tw.angular.z = float(v), float(w)
    return tw


def run(g, seconds, cmd=(0.2, 0.0), odom=True, imu=True, image=True, odom_frozen=False,
        imu_frozen=False, stamps_frozen=(), ros=True, wall=True, rtf=1.0):
    """Feed fresh inputs and tick every 20 ms for `seconds`; returns the /cmd_vel sent.

    cmd: (v, w) candidate every tick, a callable t -> (v, w) | None, or None (no candidate).
    odom/imu/image False: that input is not published. *_frozen: published with the same
    content. stamps_frozen: names whose header stamp no longer advances.
    """
    start = len(g.sent)
    clock = g.fake_clock
    for _ in range(int(round(seconds / DT))):
        clock.advance(DT, ros=ros, wall=wall, rtf=rtf)
        t = clock.t
        g.seq += 1
        if not hasattr(g, 'odom_x'):
            g.odom_x = 0.0
        if not odom_frozen:
            g.odom_x += 1e-3
        frozen_t = getattr(g, 'frozen_t', None)
        if frozen_t is None:
            g.frozen_t = frozen_t = t
        if odom:
            g.on_odom(odom_msg(t, g.odom_x, frozen_t if 'odom' in stamps_frozen else None))
        if imu:
            g.on_imu(imu_msg(t, 0 if imu_frozen else g.seq, frozen_t if 'imu' in stamps_frozen else None))
        if image:
            g.on_image(image_msg(t, frozen_t if 'image' in stamps_frozen else None))
        c = cmd(t) if callable(cmd) else cmd
        if c is not None:
            g.on_cmd(c if isinstance(c, Twist) else twist(*c))
        g.tick()
    return g.sent[start:]


def vw(tw):
    return tw.linear.x, tw.angular.z


def is_zero(tw):
    return (tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.x, tw.angular.y, tw.angular.z) == (0.0,) * 6


def last_status(g):
    return json.loads(g.status[-1].data)


# ------------------------------------------------------------------ start gate
def test_zero_before_the_start_delay_then_moves(guard):
    """No motion before 20 s after start (wall clock), even with every input fresh."""
    g = guard({'start_delay': 20.0})
    before = run(g, 19.9)
    assert before and all(is_zero(t) for t in before)
    assert g.state == 'GATE' and 'start delay' in g.reason
    after = run(g, 0.5)
    assert any(t.linear.x > 0.0 for t in after)
    assert g.state == 'RUN'


def test_start_delay_is_wall_time_even_when_sim_time_runs_faster(guard):
    """Sim time running at 2x real time must not open the gate early (20 s rule = stopwatch)."""
    g = guard({'start_delay': 20.0})
    out = run(g, 15.0, rtf=2.0)                     # 30 s of sim time, 15 s of wall time
    assert all(is_zero(t) for t in out)
    assert g.state == 'GATE'
    out = run(g, 5.5, rtf=2.0)
    assert any(t.linear.x > 0.0 for t in out)


@pytest.mark.parametrize('missing', ['odom', 'imu', 'image'])
def test_gate_waits_until_all_three_sensors_have_been_seen(guard, missing):
    g = guard()
    out = run(g, 3.0, **{missing: False})
    assert all(is_zero(t) for t in out)
    assert g.state == 'GATE' and missing in g.reason
    out = run(g, 1.0)
    assert any(t.linear.x > 0.0 for t in out)


def test_gate_opening_ramps_up_without_a_jump_and_keeps_the_curvature(guard):
    """The follower may have ramped its own speed while the gate was closed: the guard's
    acceleration limit turns the step into a ramp, with w/v (path curvature) unchanged."""
    g = guard({'start_delay': 2.0})
    acc = g.get_parameter('max_linear_accel').value
    assert acc > 0.0
    out = run(g, 4.0, cmd=(0.7, 1.4))
    moving = [vw(t) for t in out if not is_zero(t)]
    assert moving
    assert moving[0][0] <= acc * DT + 1e-9
    v = [0.0] + [m[0] for m in moving]
    assert max(b - a for a, b in zip(v, v[1:])) <= acc * DT + 1e-9
    assert all(w == pytest.approx(2.0 * v) for v, w in moving)
    assert moving[-1] == pytest.approx((0.7, 1.4))


def test_follower_that_ramped_while_blocked_starts_smoothly(guard):
    """End to end with the real follower: it plans and ramps its v_cmd while the gate is
    closed (it cannot know), the guard still starts the robot from zero."""
    from test_follower_node import odom as f_odom, straight_odom_path
    from line_follower.follower_node import FollowerNode
    f = FollowerNode()
    try:
        f.sent = []
        f.cmd_pub.publish = f.sent.append
        g = guard({'start_delay': 3.0})
        f.get_clock = lambda: g.fake_clock
        f.on_odom(f_odom(0.0, 0.0, 0.0, 0.0))
        f.path = straight_odom_path(f, 3.0)
        f.path_end = 'len'

        def follower(_t):
            f.control()
            return f.sent[-1]
        out = run(g, 4.0, cmd=follower)
        v_follower = [t.linear.x for t in f.sent]
        assert max(v_follower[:140]) == pytest.approx(f.get_parameter('v_max').value)   # ramped while blocked
        moving = [t.linear.x for t in out if not is_zero(t)]
        acc = g.get_parameter('max_linear_accel').value
        assert moving[0] <= acc * DT + 1e-9
        steps = [b - a for a, b in zip([0.0] + moving, moving)]
        assert max(steps) <= acc * DT + 1e-9
    finally:
        f.destroy_node()


# ------------------------------------------------------------------ command timeout
def test_command_timeout_stops_then_resumes_after_recover_time(guard):
    g = guard()
    timeout = g.get_parameter('cmd_timeout').value
    recover = g.get_parameter('recover_time').value
    run(g, 2.0)
    assert g.state == 'RUN'
    out = run(g, 1.0, cmd=None)                      # follower silent (crash, hang)
    first_zero = next(i for i, t in enumerate(out) if is_zero(t))
    assert first_zero * DT < timeout + 0.05
    assert all(is_zero(t) for t in out[first_zero:])
    assert g.state == 'STOP' and 'command' in g.reason
    back = run(g, 1.0)
    first_move = next(i for i, t in enumerate(back) if not is_zero(t))
    assert recover - 0.03 <= first_move * DT / 2 + DT <= recover + 0.1   # 2 outputs per step
    assert back[first_move].linear.x <= g.get_parameter('max_linear_accel').value * DT + 1e-9


def test_follower_crash_keeps_the_robot_stopped(guard):
    g = guard()
    run(g, 1.0)
    out = run(g, 30.0, cmd=None)
    assert sum(not is_zero(t) for t in out) <= 2 * int(0.3 / DT) + 2
    assert all(is_zero(t) for t in out[-1000:])


def test_no_candidate_yet_means_zero(guard):
    g = guard()
    out = run(g, 2.0, cmd=None)
    assert out and all(is_zero(t) for t in out)
    assert 'command' in g.reason


def test_zero_candidate_stops_at_once_with_the_sim_defaults(guard):
    g = guard()
    run(g, 2.0, cmd=(0.5, 0.0))
    out = run(g, 0.1, cmd=(0.0, 0.0))
    assert is_zero(out[0])


# ------------------------------------------------------------------ sensor watchdog
@pytest.mark.parametrize('name', ['odom', 'imu', 'image'])
def test_stale_sensor_stops_within_its_timeout_and_resumes(guard, name):
    g = guard()
    timeout = g.get_parameter(f'{name}_timeout').value
    recover = g.get_parameter('recover_time').value
    run(g, 2.0)
    out = run(g, 1.5, **{name: False})
    first_zero = next(i for i, t in enumerate(out) if is_zero(t))
    assert timeout - 0.02 <= first_zero * DT / 2 < timeout + 0.05
    assert all(is_zero(t) for t in out[first_zero:])
    assert g.state == 'STOP' and name in g.reason
    back = run(g, 1.0)
    first_move = next(i for i, t in enumerate(back) if not is_zero(t))
    assert recover - 0.03 <= first_move * DT / 2 <= recover + 0.1


def test_a_short_camera_hiccup_does_not_stop(guard):
    g = guard()
    run(g, 2.0)
    out = run(g, 0.3, image=False)                  # 0.3 s without frames < 0.5 s timeout
    assert not any(is_zero(t) for t in out)
    assert g.state == 'RUN'


@pytest.mark.parametrize('name', ['odom', 'imu', 'image'])
def test_messages_whose_stamp_stops_advancing_count_as_stale(guard, name):
    """A driver that keeps republishing its last sample (same header stamp)."""
    g = guard()
    run(g, 2.0)
    g.frozen_t = g.fake_clock.t
    out = run(g, 1.5, stamps_frozen=(name,))
    assert all(is_zero(t) for t in out[-40:])
    assert g.state == 'STOP' and name in g.reason and 'stamp' in g.reason


def test_odom_with_unchanged_content_while_driving_stops(guard):
    """New stamps, same pose: wheel odometry stuck while the robot is commanded to move."""
    g = guard()
    frozen = g.get_parameter('frozen_timeout').value
    run(g, 2.0)
    out = run(g, frozen + 0.5, odom_frozen=True)
    assert all(is_zero(t) for t in out[-10:])
    assert g.state == 'STOP' and 'odom' in g.reason
    # stays stopped (no stop-and-go) until the pose changes again
    out = run(g, 3.0, odom_frozen=True)
    assert all(is_zero(t) for t in out)
    out = run(g, 1.0)
    assert g.state == 'RUN'


def test_odom_with_unchanged_content_while_standing_still_is_fine(guard):
    g = guard()
    out = run(g, 5.0, cmd=(0.0, 0.0), odom_frozen=True)
    assert all(is_zero(t) for t in out)
    assert g.state == 'RUN'


def test_imu_with_unchanged_content_stops(guard):
    """A real or simulated IMU always has noise: bit-identical samples mean a frozen source."""
    g = guard()
    run(g, 2.0)
    out = run(g, g.get_parameter('frozen_timeout').value + 0.5, imu_frozen=True)
    assert all(is_zero(t) for t in out[-10:])
    assert g.state == 'STOP' and 'imu' in g.reason


def test_stopped_sim_clock_stops(guard):
    """/clock stops (bridge dead / simulator hung): sim-time ages freeze, wall time does not."""
    g = guard()
    limit = g.get_parameter('clock_timeout').value
    run(g, 2.0)
    out = run(g, limit - 0.1, ros=False)              # sim pauses shorter than that are fine
    assert not any(is_zero(t) for t in out)
    out = run(g, 0.5, ros=False)
    assert all(is_zero(t) for t in out[-10:])
    assert g.state == 'STOP' and 'clock' in g.reason


def test_slow_simulation_is_not_a_fault(guard):
    """Real-time factor 0.3: ages are measured in sim time, so nothing is stale."""
    g = guard()
    out = run(g, 6.0, rtf=0.3)
    assert g.state == 'RUN'
    assert not any(is_zero(t) for t in out[-100:])


def test_long_outage_latches_with_the_real_config(guard):
    g = guard(config='safety_real.yaml')
    after = g.get_parameter('latch_after').value
    assert after > 0.0
    run(g, 2.0)
    run(g, after + 1.0, image=False)
    assert g.state == 'LATCHED'
    out = run(g, 3.0)
    assert all(is_zero(t) for t in out)
    assert g.state == 'LATCHED'


def test_sim_config_never_latches_on_stale_inputs(guard):
    g = guard()
    assert g.get_parameter('latch_after').value == 0.0
    run(g, 2.0)
    run(g, 20.0, image=False)
    run(g, 1.0)
    assert g.state == 'RUN'


# ------------------------------------------------------------------ non-finite commands
@pytest.mark.parametrize('field, value', [('linear.x', math.nan), ('angular.z', math.inf),
                                          ('linear.x', -math.inf), ('linear.y', math.nan)])
def test_non_finite_command_stops_and_latches_until_restart(guard, field, value):
    g = guard()
    run(g, 2.0)
    bad = twist(0.2, 0.1)
    part, axis = field.split('.')
    setattr(getattr(bad, part), axis, value)
    g.fake_clock.advance(DT)
    g.on_cmd(bad)
    assert is_zero(g.sent[-1])
    assert g.state == 'LATCHED' and 'finite' in g.reason
    out = run(g, 5.0)
    assert all(is_zero(t) for t in out)
    assert all(math.isfinite(v) for t in g.sent for v in vw(t))


# ------------------------------------------------------------------ limits
@pytest.mark.parametrize('v, w, max_v, max_w, expected', [
    (0.5, 1.0, 0.26, 1.82, (0.26, 0.52)),            # speed-bound
    (0.1, 3.0, 0.26, 1.82, (0.1 * 1.82 / 3.0, 1.82)),  # turn-bound
    (0.7, 3.0, 0.26, 1.82, (0.26, 3.0 * 0.26 / 0.7)),  # both over: the tighter wins
    (-0.5, 1.0, 0.26, 1.82, (-0.26, 0.52)),          # reverse
    (0.0, -2.5, 0.26, 1.82, (0.0, -1.82)),           # spin in place
    (0.2, 1.0, 0.26, 1.82, (0.2, 1.0)),              # inside: unchanged
])
def test_scale_to_limits_preserves_curvature(v, w, max_v, max_w, expected):
    out = scale_to_limits(v, w, max_v, max_w)
    assert out == pytest.approx(expected)
    if v and out[0]:
        assert out[1] / out[0] == pytest.approx(w / v)


def test_sim_defaults_pass_lap_like_commands_unchanged(guard):
    """Speeds up to 0.7 m/s, |w| up to 3 rad/s, 0.6 m/s^2 ramps, sudden braking and w jumps
    of 3 rad/s in one tick (all seen on the official lap): the sim config must not touch them."""
    g = guard()
    seq = []
    v = 0.0
    for k in range(600):
        if k < 60:
            v = min(0.7, v + 0.6 * DT)
        elif k == 300:
            v = 0.06                               # sharp corner: v drops to v_min at once
        elif k > 300:
            v = min(0.45, v + 0.6 * DT)
        w = 3.0 * math.sin(k / 7.0) if k % 50 else -3.0
        seq.append((v, w))
    run(g, 1.0, cmd=(0.0, 0.0))
    it = iter(seq)
    sent = []

    def cmd(_t):
        c = next(it)
        sent.append(c)
        return c
    out = run(g, len(seq) * DT, cmd=cmd)
    applied = [x for t in out[0::2] for x in vw(t)]   # on_cmd output; tick re-sends it
    assert applied == pytest.approx([x for c in sent for x in c], abs=1e-12)


def test_real_config_clamps_speed_and_turn_rate_together(guard):
    g = guard(config='safety_real.yaml', params={'start_delay': 0.0})
    assert g.get_parameter('max_linear').value == pytest.approx(0.26)
    assert g.get_parameter('max_angular').value == pytest.approx(1.82)
    out = run(g, 5.0, cmd=(0.7, 3.0))
    assert all(abs(t.linear.x) <= 0.26 + 1e-9 and abs(t.angular.z) <= 1.82 + 1e-9 for t in out)
    assert vw(out[-1]) == pytest.approx((0.26, 3.0 * 0.26 / 0.7))
    for t in out:
        if t.linear.x > 1e-6:
            assert t.angular.z / t.linear.x == pytest.approx(3.0 / 0.7)


def test_real_config_acceleration_limits(guard):
    g = guard(config='safety_real.yaml', params={'start_delay': 0.0})
    acc = g.get_parameter('max_linear_accel').value
    dec = g.get_parameter('max_linear_decel').value
    wacc = g.get_parameter('max_angular_accel').value
    assert acc > 0 and dec > 0 and wacc > 0
    up = [vw(t) for t in run(g, 3.0, cmd=(0.26, 0.0))]
    dv = [b[0] - a[0] for a, b in zip([(0.0, 0.0)] + up, up)]
    assert max(dv) <= acc * DT + 1e-9 and up[-1][0] == pytest.approx(0.26)
    down = [vw(t) for t in run(g, 2.0, cmd=(0.0, 0.0))]
    dv = [a[0] - b[0] for a, b in zip([up[-1]] + down, down)]
    assert max(dv) <= dec * DT + 1e-9 and down[-1][0] == pytest.approx(0.0)
    turn = [vw(t) for t in run(g, 3.0, cmd=(0.0, 1.5))]
    dw = [b[1] - a[1] for a, b in zip([down[-1]] + turn, turn)]
    assert max(dw) <= wacc * DT + 1e-9 and turn[-1][1] == pytest.approx(1.5)


def test_safety_stop_is_immediate_even_with_a_deceleration_limit(guard):
    g = guard(config='safety_real.yaml', params={'start_delay': 0.0})
    run(g, 3.0, cmd=(0.26, 0.0))
    out = run(g, 1.0, cmd=(0.26, 0.0), odom=False)
    first_zero = next(i for i, t in enumerate(out) if is_zero(t))
    assert all(t.linear.x == pytest.approx(0.26) for t in out[:first_zero])


# ------------------------------------------------------------------ spin budget
def test_turning_in_place_longer_than_the_budget_stops_and_latches(guard):
    g = guard()
    budget = g.get_parameter('spin_budget').value
    assert budget >= 30.0                           # sim: generous, the search helps scoring
    run(g, 1.0)
    out = run(g, budget - 1.0, cmd=(0.0, 0.8))
    assert not any(is_zero(t) for t in out)
    out = run(g, 2.0, cmd=(0.0, 0.8))
    assert all(is_zero(t) for t in out[-40:])
    assert g.state == 'LATCHED' and 'spin' in g.reason
    out = run(g, 2.0, cmd=(0.2, 0.0))
    assert all(is_zero(t) for t in out)


def test_real_config_has_a_tighter_spin_budget(guard):
    g = guard(config='safety_real.yaml', params={'start_delay': 0.0})
    budget = g.get_parameter('spin_budget').value
    assert 0.0 < budget <= 10.0
    run(g, budget + 2.0, cmd=(0.0, 0.8))
    assert g.state == 'LATCHED' and 'spin' in g.reason


def test_spin_budget_restarts_after_real_forward_travel(guard):
    g = guard()
    budget = g.get_parameter('spin_budget').value
    reset = g.get_parameter('spin_reset_distance').value
    for _ in range(3):
        run(g, budget * 0.7, cmd=(0.0, 0.8))
        run(g, reset / 0.2 + 0.5, cmd=(0.2, 0.3))    # drives on (radius 0.67 m): budget restarts
    assert g.state == 'RUN'


def test_spinning_broken_by_short_arcs_still_uses_up_the_budget(guard):
    """Measured in Gazebo (guard_sigkill r2): w = +-3 rad/s at v = 0.06 for up to 2.8 s, broken
    by 0.3 s arcs of radius ~0.1 m, for 220 s inside a 0.5 m circle. The arcs are not progress."""
    g = guard()
    budget = g.get_parameter('spin_budget').value
    run(g, 1.0)
    k = 0
    while g.state == 'RUN' and k < 40:
        k += 1
        run(g, 2.8, cmd=(0.06, 3.0 if k % 2 else -3.0))
        run(g, 0.3, cmd=(0.2, 2.0))
    assert g.state == 'LATCHED' and 'spin' in g.reason
    assert budget / 3.0 <= k <= budget / 2.8 + 1          # latched once ~budget s were spent spinning


def test_a_very_tight_turn_counts_as_spinning(guard):
    """v = 0.06 m/s with w = 3 rad/s is a 2 cm radius: turning on the spot (frozen-IMU case)."""
    g = guard({'spin_budget': 5.0})
    run(g, 6.0, cmd=(0.06, -3.0))
    assert g.state == 'LATCHED' and 'spin' in g.reason


# ------------------------------------------------------------------ status topic
def test_status_topic_reports_state_and_reason(guard):
    g = guard({'start_delay': 1.0})
    run(g, 0.5)
    s = last_status(g)
    assert s['state'] == 'GATE' and 'start delay' in s['reason']
    run(g, 1.5)
    assert last_status(g)['state'] == 'RUN'
    run(g, 1.0, odom=False)
    s = last_status(g)
    assert s['state'] == 'STOP' and 'odom' in s['reason']
    assert s['v'] == 0.0 and s['w'] == 0.0
    assert set(s['age']) >= {'cmd', 'odom', 'imu', 'image'}


def test_status_is_published_on_every_change(guard):
    g = guard()
    run(g, 1.0)
    n = len(g.status)
    g.fake_clock.advance(DT)
    g.on_cmd(twist(math.nan, 0.0))
    assert len(g.status) == n + 1 and last_status(g)['state'] == 'LATCHED'


# ------------------------------------------------------------------ zero on exit
def test_shutdown_publishes_zero_several_times(guard):
    g = guard()
    run(g, 2.0)
    assert not is_zero(g.sent[-1])
    n = len(g.sent)
    g.shutdown_zero()
    tail = g.sent[n:]
    assert len(tail) >= 3 and all(is_zero(t) for t in tail)
    assert g.state == 'EXIT' and last_status(g)['state'] == 'EXIT'


# ------------------------------------------------------------------ config + launch
@pytest.mark.parametrize('name', ['safety_sim.yaml', 'safety_real.yaml'])
def test_config_files_only_set_declared_parameters(guard, name):
    g = guard()
    params = yaml_params(name)
    assert all(g.has_parameter(k) for k in params), [k for k in params if not g.has_parameter(k)]
    for key, value in params.items():
        if key != 'use_sim_time':
            assert type(value) is type(g.get_parameter(key).value), key


def test_sim_config_limits_are_above_what_the_follower_commands():
    sim = yaml_params('safety_sim.yaml')
    rule = yaml.safe_load((CONFIG / 'rule.yaml').read_text())['follower_node']['ros__parameters']
    assert sim['max_linear'] >= max(rule['speed_profile'][2::3] + [rule['v_max']])
    assert sim['max_angular'] >= rule['omega_max']
    assert sim['spin_budget'] >= 30.0 and sim['latch_after'] == 0.0


def test_launch_runs_the_guard_and_remaps_the_follower():
    text = (PKG / 'launch' / 'line_follower.launch.py').read_text()
    assert "executable='safety_guard'" in text
    assert "('/cmd_vel', '/line_follower/cmd_raw')" in text
    setup = (PKG / 'setup.py').read_text()
    assert 'safety_guard = line_follower.safety_guard:main' in setup


# ------------------------------------------------------------------ real processes + signals
def _child_env():
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(rclpy.get_default_context().get_domain_id())
    src = str(Path(line_follower.__file__).resolve().parents[1])
    env['PYTHONPATH'] = os.pathsep.join(filter(None, [src, env.get('PYTHONPATH')]))
    env['PYTHONUNBUFFERED'] = '1'
    return env


def _signal_test(module, ros_args, listen, sig, feed_cmd):
    """Start `module`.main() in a child, feed it, wait for a moving command on `listen`,
    send `sig` while the inputs keep coming, and return (exit code, messages after the
    signal, log text)."""
    ns = f'sgt{os.getpid()}_{module}_{int(sig)}'
    remap = ['-r', f'/odom:=/{ns}/odom', '-r', f'/imu:=/{ns}/imu', '-r', f'/camera/image_raw:=/{ns}/image',
             '-r', f'/line_follower/debug:=/{ns}/debug', '-r', f'/line_follower/safety:=/{ns}/safety']
    node = rclpy.create_node(f'{ns}_probe')
    got = []
    sub_topic = f'/{ns}/{listen}'
    node.create_subscription(Twist, sub_topic, got.append, 50)
    pubs = {name: node.create_publisher(kind, f'/{ns}/{name}', 10)
            for name, kind in (('odom', Odometry), ('imu', Imu), ('image', Image), ('cmd_raw', Twist))}
    log = tempfile.TemporaryFile(mode='w+')
    proc = subprocess.Popen(
        [sys.executable, '-c', f'from line_follower.{module} import main; main()', '--ros-args',
         '-p', 'use_sim_time:=false', *ros_args(ns), *remap],
        env=_child_env(), stdout=log, stderr=subprocess.STDOUT)
    k = 0
    try:
        def feed():
            nonlocal k
            k += 1
            t = time.time()
            m = odom_msg(t, 1e-3 * k)
            pubs['odom'].publish(m)
            pubs['imu'].publish(imu_msg(t, k))
            pubs['image'].publish(image_msg(t))
            if feed_cmd:
                pubs['cmd_raw'].publish(twist(0.2, 0.1))
            rclpy.spin_once(node, timeout_sec=0.02)

        deadline = time.monotonic() + 40.0
        while not any(not is_zero(t) for t in got):
            assert proc.poll() is None, 'child exited early'
            assert time.monotonic() < deadline, 'no moving command from the child'
            feed()
        n = len(got)
        proc.send_signal(sig)
        deadline = time.monotonic() + 15.0
        while proc.poll() is None and time.monotonic() < deadline:
            feed()
        for _ in range(25):                          # late deliveries
            rclpy.spin_once(node, timeout_sec=0.02)
        code = proc.poll()
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
        node.destroy_node()
    log.seek(0)
    return code, got[n:], log.read()


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM])
def test_guard_process_publishes_zero_on_signal(ros, sig):
    code, after, text = _signal_test(
        'safety_guard', lambda ns: ['-p', 'start_delay:=0.0', '-r', f'/cmd_vel:=/{ns}/cmd_vel',
                                    '-r', f'/line_follower/cmd_raw:=/{ns}/cmd_raw'],
        'cmd_vel', sig, feed_cmd=True)
    assert code == 0, text
    assert after and is_zero(after[-1]), text
    assert sum(is_zero(t) for t in after) >= 3, text


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM])
def test_follower_process_publishes_zero_on_signal(ros, sig):
    """The follower alone (as run_user.py runs it): its last command must be zero."""
    code, after, text = _signal_test(
        'follower_node', lambda ns: ['-r', f'/cmd_vel:=/{ns}/cmd_out'], 'cmd_out', sig, feed_cmd=False)
    assert code == 0, text
    assert after and is_zero(after[-1]), text


def test_module_exports():
    assert safety_guard.CMD_TOPIC == '/line_follower/cmd_raw'
    assert safety_guard.STATUS_TOPIC == '/line_follower/safety'
