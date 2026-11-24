"""판단 (planning: finish judgement, lost-line recovery) + node plumbing: `FollowerNode`.

Also covers the 인지 pose-estimation part that lives in the node (on_odom dead
reckoning, on_imu pitch filter).

The node is created for real (rclpy, default parameters) but never spun: callbacks
and control() are called directly with hand-made Odometry / Imu messages, the clock
is replaced by a fake one that advances 20 ms per control tick, and /cmd_vel is
captured by replacing node.cmd_pub.publish.

Pinned current behaviour (not a judgement that it is right):
  * no path: creep straight at v_min for 0.35 m, then rotate in place at 0.8 rad/s
    FOREVER - there is no give-up / stop timeout.
  * no stale-sensor watchdog: if /odom (or the camera) stops, control() keeps
    commanding the last path at speed (SAFETY GAP xfail below).
"""
import math
import os

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import synth
from line_follower import follower_node, line_map
from line_follower.follower_node import FollowerNode

DT = 0.02
paint = synth.to_grid_painter(line_map)


# ------------------------------------------------------------------ fixtures / helpers
@pytest.fixture(scope='module')
def ros():
    owns = not rclpy.ok()
    if owns:
        # CI (91) and `lab_env.sh test` (78) set their own test domain; without one, pick a
        # private domain clear of 0 and of the lab simulation domains 77-85
        env = os.environ.get('LF_TEST_DOMAIN_ID') or os.environ.get('ROS_DOMAIN_ID')
        rclpy.init(domain_id=int(env) if env else 30 + os.getpid() % 40)
    yield
    if owns:
        rclpy.shutdown()


class FakeClock:
    def __init__(self):
        self.ns = 1_000_000_000

    def now(self):
        return type('T', (), {'nanoseconds': self.ns})()


@pytest.fixture
def node(ros):
    n = FollowerNode()
    n.sent = []
    n.cmd_pub.publish = n.sent.append
    n.fake_clock = FakeClock()
    n.get_clock = lambda: n.fake_clock
    yield n
    n.destroy_node()


def quat(yaw=0.0, pitch=0.0):
    """ROS quaternion (x, y, z, w) for roll 0 and the given ROS pitch / yaw."""
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return -sp * sy, sp * cy, cp * sy, cp * cy


def odom(x, y, yaw, t):
    m = Odometry()
    m.header.stamp.sec, m.header.stamp.nanosec = int(t), int(round((t % 1) * 1e9))
    m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
    q = m.pose.pose.orientation
    q.x, q.y, q.z, q.w = quat(yaw)
    return m


def imu(yaw=0.0, nose_up=0.0, t=0.0, pitch_rate=0.0):
    m = Imu()
    m.header.stamp.sec, m.header.stamp.nanosec = int(t), int(round((t % 1) * 1e9))
    q = m.orientation
    q.x, q.y, q.z, q.w = quat(yaw, -nose_up)      # ROS pitch is nose-down positive
    m.angular_velocity.y = pitch_rate
    return m


def tick(n, k=1):
    """k control ticks of 20 ms; returns the Twists published during them."""
    start = len(n.sent)
    for _ in range(k):
        n.fake_clock.ns += int(DT * 1e9)
        n.control()
    return n.sent[start:]


def is_zero(tw):
    return (tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.x, tw.angular.y, tw.angular.z) == (0.0,) * 6


def straight_odom_path(n, length=1.0, start=0.0):
    k = int(round(length / 0.02))
    return [n.to_odom(start + 0.02 * i, 0.0) for i in range(k + 1)]


# ------------------------------------------------------------------ 인지: pose estimation
def test_first_odom_message_sets_the_pose(node):
    assert node.pose is None
    node.on_odom(odom(1.0, 2.0, 0.5, 0.0))
    assert node.pose == pytest.approx((1.0, 2.0, 0.5))
    assert node.travel == 0.0


def test_dead_reckoning_straight_drive_without_imu(node):
    yaw = 0.5
    for i in range(51):
        d = 0.01 * i
        node.on_odom(odom(1.0 + d * math.cos(yaw), 2.0 + d * math.sin(yaw), yaw, 0.02 * i))
    assert node.pose == pytest.approx((1.0 + 0.5 * math.cos(yaw), 2.0 + 0.5 * math.sin(yaw), yaw), abs=1e-9)
    assert node.travel == pytest.approx(0.5, abs=1e-9)
    assert node.yaw_off is None


def test_heading_comes_from_the_imu_with_an_offset_taken_at_first_sight(node):
    """/odom says 'straight along +x' all the time (wheels slip); the IMU says the robot
    turned 90 deg left. The pose follows the IMU, re-zeroed to the odom yaw at first sight."""
    node.on_odom(odom(0.0, 0.0, 0.0, 0.0))
    node.on_imu(imu(yaw=1.0))                       # IMU frame differs from odom by 1 rad
    node.on_odom(odom(0.01, 0.0, 0.0, 0.02))
    assert node.yaw_off == pytest.approx(-1.0)
    assert node.pose == pytest.approx((0.01, 0.0, 0.0), abs=1e-12)
    node.on_imu(imu(yaw=1.0 + math.pi / 2))
    for i in range(2, 22):
        node.on_odom(odom(0.01 * i, 0.0, 0.0, 0.02 * i))
    x, y, yaw = node.pose
    assert yaw == pytest.approx(math.pi / 2)
    assert node.yaw_off == pytest.approx(-1.0)      # taken once, not re-zeroed
    # first step after the turn uses the mid heading (45 deg), then straight along +y
    assert x == pytest.approx(0.01 + 0.01 * math.cos(math.pi / 4), abs=1e-9)
    assert y == pytest.approx(0.01 * math.sin(math.pi / 4) + 0.19, abs=1e-9)
    assert node.travel == pytest.approx(0.21, abs=1e-9)


def test_travel_accumulates_forward_and_reverse_distance(node):
    t = 0.0
    for i in range(51):                              # 0.5 m forward
        node.on_odom(odom(0.01 * i, 0.0, 0.0, t))
        t += 0.02
    for i in range(1, 21):                           # 0.2 m backwards, same heading
        node.on_odom(odom(0.5 - 0.01 * i, 0.0, 0.0, t))
        t += 0.02
    assert node.pose[0] == pytest.approx(0.3, abs=1e-9)
    assert node.travel == pytest.approx(0.7, abs=1e-9)


def test_imu_pitch_filter(node):
    up = math.radians(5.0)
    for k in range(1, 31):
        node.on_imu(imu(nose_up=up, t=0.005 * k))
        assert node.pitch_f == pytest.approx(up * (1.0 - 0.9 ** k), rel=1e-9)
    assert node.pitch_f > math.radians(3.0)
    assert node.imu[-1][1] == pytest.approx(up)      # nose-up is positive
    # on the hill (|pitch_f| > 3 deg) the camera pitch falls back to the rest value
    assert node.pitch_at(0.15) == pytest.approx(follower_node.REST_PITCH)


def test_pitch_at_uses_the_nearest_imu_sample_on_flat_ground(node):
    for k in range(10):
        node.on_imu(imu(nose_up=math.radians(0.2 * k), t=1.0 + 0.01 * k))
    assert abs(node.pitch_f) < math.radians(3.0)
    assert node.pitch_at(1.031) == pytest.approx(math.radians(0.6))


# ------------------------------------------------------------------ 판단: finish gate
def _replan_with_t(n, monkeypatch, travel, verdict=None):
    n.on_odom(odom(0.0, 0.0, 0.0, 0.0))
    paint(n.map.grid, [[(0.30, 0.0), (1.0, 0.0)]])          # line ahead: acquire() finds it
    pts = [(0.30 + 0.02 * i, 0.0) for i in range(21)]
    monkeypatch.setattr(n.map, 'trace', lambda X, Y, h, **kw: (list(pts), 't'))
    if verdict is not None:
        monkeypatch.setattr(n.controller, 'confirm_finish', lambda path, obs: verdict)
    n.travel = travel
    n.replan()
    return pts


def test_finish_t_is_ignored_before_finish_min_dist(node, monkeypatch):
    limit = node.get_parameter('finish_min_dist').value
    assert limit == 10.0
    _replan_with_t(node, monkeypatch, travel=limit - 0.5)
    assert node.path_end == 't'
    assert node.t_point is None
    assert len(node.path) >= 21


def test_finish_t_is_accepted_after_finish_min_dist(node, monkeypatch):
    pts = _replan_with_t(node, monkeypatch, travel=node.get_parameter('finish_min_dist').value + 0.5)
    assert node.t_point == pts[-1]


@pytest.mark.parametrize('verdict, travel, accepted', [(True, 0.0, True), (False, 30.0, False)])
def test_controller_confirm_finish_overrides_the_distance_rule(node, monkeypatch, verdict, travel, accepted):
    _replan_with_t(node, monkeypatch, travel=travel, verdict=verdict)
    assert (node.t_point is not None) == accepted


def test_reaching_the_t_point_stops_and_stays_stopped(node):
    node.on_odom(odom(0.0, 0.0, 0.0, 0.0))
    node.path = straight_odom_path(node, 0.3)
    node.path_end = 't'
    node.t_point = (0.25, 0.0)
    moving = tick(node, 1)
    assert moving[0].linear.x > 0.0 and not node.done
    node.t_point = (0.005, 0.0)                     # now just ahead of the axle: arrived
    out = tick(node, 1)
    assert node.done
    assert len(out) == 1 and is_zero(out[0])
    assert node.v_cmd == node.w_cmd == 0.0
    later = tick(node, 50)
    assert len(later) == 50 and all(is_zero(t) for t in later)


# ------------------------------------------------------------------ 판단: lost-line recovery
def test_no_path_creeps_then_rotates_forever(node):
    """Pinned current behaviour: creep v_min for ~0.35 m, then w = 0.8 rad/s with no
    timeout - the robot never stops by itself while it has no path."""
    node.on_odom(odom(0.0, 0.0, 0.0, 0.0))
    v_min = node.get_parameter('v_min').value
    out = tick(node, 3000)                          # 60 s
    creep = [t for t in out if t.linear.x == pytest.approx(v_min) and t.angular.z == 0.0]
    n_creep = len(creep)
    assert out[:n_creep] == creep                   # creep first ...
    assert n_creep * v_min * DT == pytest.approx(0.35, abs=0.003)     # 292 ticks = 0.3504 m
    rest = out[n_creep:]
    assert all(t.linear.x == 0.0 and t.angular.z == pytest.approx(0.8) for t in rest)   # ... then spin
    assert not any(is_zero(t) for t in out)
    assert node.blind == pytest.approx(n_creep * v_min * DT, abs=1e-9)


def test_a_new_path_resets_the_blind_distance(node):
    node.on_odom(odom(0.0, 0.0, 0.0, 0.0))
    tick(node, 100)
    assert node.blind > 0.0
    node.path = straight_odom_path(node, 0.6)
    out = tick(node, 1)
    assert node.blind == 0.0 and out[0].linear.x > 0.0


# ------------------------------------------------------------------ node: driving / stopping
def test_control_does_nothing_before_the_first_odom(node):
    assert tick(node, 5) == []


def test_control_publishes_the_controller_command(node):
    node.on_odom(odom(0.0, 0.0, 0.0, 0.0))
    node.path = straight_odom_path(node, 0.8)
    node.path_end = 'len'
    out = tick(node, 30)
    assert len(out) == 30
    assert all(t.angular.z == 0.0 for t in out)
    assert out[0].linear.x == pytest.approx(node.get_parameter('v_min').value)
    assert out[-1].linear.x == pytest.approx(node.get_parameter('v_max').value)
    assert (node.v_cmd, node.w_cmd) == (out[-1].linear.x, out[-1].angular.z)


def test_stop_publishes_zero(node):
    node.v_cmd, node.w_cmd = 0.3, 0.5
    node.stop()
    assert len(node.sent) == 1 and isinstance(node.sent[0], Twist) and is_zero(node.sent[0])
    assert node.v_cmd == node.w_cmd == 0.0


@pytest.mark.xfail(strict=True, reason='SAFETY GAP: no stale-odometry/camera watchdog')
def test_stale_odometry_stops_the_robot_within_half_a_second(node):
    """/odom flows for 0.5 s while driving a known path, then stops arriving. A safe node
    would stop within 0.5 s; this one keeps commanding the last path at speed."""
    node.on_odom(odom(0.0, 0.0, 0.0, 0.0))
    node.path = straight_odom_path(node, 2.0)
    node.path_end = 'len'
    x, t = 0.0, 0.0
    for _ in range(25):                             # 0.5 s of normal driving
        tick(node, 1)
        x += node.v_cmd * DT
        t += DT
        node.on_odom(odom(x, 0.0, 0.0, t))
    after = tick(node, 75)                          # 1.5 s without any /odom
    v_late = [tw.linear.x for tw in after[25:]]     # from 0.5 s after the last message
    print(f'last odom at x={x:.3f} m; commanded v 0.5..1.5 s later: '
          f'min {min(v_late):.3f} max {max(v_late):.3f} m/s')
    assert all(v == 0.0 for v in v_late)
