"""인지 기록 + 판단: the odom-frame line map (`line_map.LineMap`).

인지 (perception, line recording): `update` writes the BEV line mask into the 5 mm
odom grid at the robot pose and overwrites what the camera sees; `hit` / `snap` read it.
판단 (planning, path choice): `trace` follows the line - straightest branch at a
crossing, 't' at a T bar (the finish), `_bridge` over short gaps, 'open' otherwise.

Lines are either produced through the real BEV mask (`synth.truth_bev`) or painted
straight into the grid with the same "within 1 cm" rule (`synth.to_grid_painter`).

Measured behaviour pinned here (rp/ros2:jazzy-lab):
  * a 2 cm line along +x is followed with 0 lateral error, a 90 deg corner with 0.2 mm
  * T bar with arms >= 5 cm -> 't' at every bar phase tried; +-3 cm arms -> not a T
  * gaps: visible gaps of 3 / 5 / 7 cm are bridged at every phase, 8 / 9 / 15 cm never
    (the 9 cm bridge radius is measured from the last traced point, which stops
    2-3 cm before the end of the line)
  * the grid is 12 m wide but `update` needs the whole 520-cell patch inside it, so it
    stops writing once |x| or |y| of the robot exceeds 4.70 m (KNOWN BUG xfail)
"""
import math

import numpy as np
import pytest

import synth
from line_follower import line_map, vision
from line_follower.line_map import LineMap

paint = synth.to_grid_painter(line_map)
_, VALID = vision.bev_warp(np.zeros((vision.H, vision.W), np.uint8))


def to_odom(pose, x, y):
    px, py, th = pose
    return px + x * math.cos(th) - y * math.sin(th), py + x * math.sin(th) + y * math.cos(th)


def painted(*polylines, **kw):
    m = LineMap()
    paint(m.grid, [np.asarray(p, float) for p in polylines], **kw)
    return m


def trace(m, X=0.0, Y=0.0, h=0.0, **kw):
    pts, end = m.trace(X, Y, h, **kw)
    return np.array(pts), end


def heading(pts):
    d = pts[-1] - pts[-5]
    return math.atan2(d[1], d[0])


# ------------------------------------------------------------------ 인지: update / hit
@pytest.mark.parametrize('yaw_deg', [0.0, 90.0, -135.0])
def test_update_puts_the_line_at_the_right_odom_cells(yaw_deg):
    pose = {0.0: (0.0, 0.0, 0.0), 90.0: (1.0, -0.5, math.pi / 2),
            -135.0: (-0.7, 0.4, math.radians(-135.0))}[yaw_deg]
    m = LineMap()
    m.update(synth.truth_bev([synth.straight(0.10)]), VALID, pose)   # line 10 cm to the LEFT
    xs = np.arange(0.36, 0.80, 0.02)
    assert all(m.hit(*to_odom(pose, x, 0.10)) for x in xs)
    assert not any(m.hit(*to_odom(pose, x, 0.13)) for x in xs)       # 3 cm beside it
    assert not any(m.hit(*to_odom(pose, x, -0.10)) for x in xs)      # mirrored side
    assert 480 <= (m.grid > 0).sum() <= 540                           # measured 510 / 510 / 504


def test_update_writes_only_the_seen_area():
    m = LineMap()
    m.update(np.full((vision.BH, vision.BW), 255, np.uint8), VALID, (0.0, 0.0, 0.0))
    written = (m.grid > 0).sum()
    assert written == pytest.approx((VALID > 0).sum(), rel=0.03)
    assert m.hit(0.6, 0.0) and m.hit(0.6, 0.25)
    assert not m.hit(0.2, 0.0)            # blind spot in front of the camera
    assert not m.hit(0.6, 0.35)           # outside the field of view at 0.6 m
    assert not m.hit(0.9, 0.0)            # beyond X_FAR


def test_later_frame_erases_line_where_it_is_seen_empty():
    line = synth.truth_bev([synth.straight(0.10)])
    empty = np.zeros_like(line)
    m = LineMap()
    m.update(line, VALID, (0.0, 0.0, 0.0))
    m.update(empty, VALID, (0.0, 0.0, 0.0))
    assert not m.grid.any()

    m = LineMap()
    m.update(line, VALID, (0.0, 0.0, 0.0))
    m.update(empty, VALID, (0.3, 0.0, 0.0))     # 30 cm further on: sees x >= ~0.63 empty
    assert m.hit(0.5, 0.1) and m.hit(0.6, 0.1)  # no longer seen: remembered
    assert not m.hit(0.65, 0.1) and not m.hit(0.7, 0.1)


def test_hit_outside_the_grid_is_false():
    m = LineMap()
    m.grid[:] = 255
    assert m.hit(0.0, 0.0)
    assert not m.hit(6.5, 0.0) and not m.hit(0.0, -6.5)


def test_update_still_maps_at_4_7_m_from_start():
    m = LineMap()
    m.update(synth.truth_bev([synth.straight(0.10)]), VALID, (4.7, 0.0, 0.0))
    assert m.hit(4.7 + 0.6, 0.10)


@pytest.mark.xfail(strict=True, reason='KNOWN BUG: fixed 12 m grid stops mapping ~4.7 m from start')
@pytest.mark.parametrize('pose', [(4.8, 0.0, 0.0), (0.0, -4.75, -math.pi / 2), (-5.0, 3.0, math.pi)])
def test_update_maps_the_line_far_from_start(pose):
    m = LineMap()
    m.update(synth.truth_bev([synth.straight(0.0)]), VALID, pose)
    assert m.hit(*to_odom(pose, 0.6, 0.0)), 'update was a silent no-op'


# ------------------------------------------------------------------ 인지: snap
def test_snap_returns_the_nearby_line_centre():
    m = painted([[0.0, 0.02], [1.0, 0.02]])             # line centre at y = 2 cm
    for qy in (0.05, 0.02, -0.01):
        sx, sy = m.snap(0.5, qy)
        assert sx == pytest.approx(0.5, abs=0.003)
        # the centroid of the 7 cells nearest the query: biased <= 2.5 mm to the query side
        assert sy == pytest.approx(0.02, abs=0.0026)
    assert m.snap(0.5, 0.2) is None                     # nothing within 6 cm
    assert m.snap(0.5, 0.06, radius=0.02) is None       # nearest edge 3 cm away > radius
    assert m.snap(0.5, 0.045, radius=0.02) is not None  # nearest edge 1.5 cm away
    assert m.snap(6.2, 0.0) is None                     # window leaves the grid


# ------------------------------------------------------------------ 판단: trace
@pytest.mark.parametrize('heading_deg', [0.0, 30.0, -120.0])
def test_trace_straight_line(heading_deg):
    h = math.radians(heading_deg)
    m = painted([[-0.1 * math.cos(h), -0.1 * math.sin(h)], [2.0 * math.cos(h), 2.0 * math.sin(h)]])
    pts, end = trace(m, 0.0, 0.0, h)
    assert end == 'len'
    assert len(pts) == 41                               # 0.8 m in 2 cm steps
    off = np.abs(-pts[:, 0] * math.sin(h) + pts[:, 1] * math.cos(h))
    assert off.max() < 0.005
    assert math.hypot(*pts[-1]) == pytest.approx(0.8, abs=0.01)
    assert math.degrees(heading(pts) - h) == pytest.approx(0.0, abs=1.0)


@pytest.mark.parametrize('side', [1.0, -1.0])
def test_trace_follows_a_90_degree_corner(side):
    m = painted([[-0.1, 0.0], [0.3, 0.0], [0.3, side * 1.0]])
    pts, end = trace(m)
    assert end == 'len'
    assert pts[-1] == pytest.approx([0.30, side * 0.51], abs=0.01)
    after = pts[np.abs(pts[:, 1]) > 0.1]
    assert np.abs(after[:, 0] - 0.3).max() < 0.002
    assert math.degrees(heading(pts)) == pytest.approx(side * 90.0, abs=2.0)


@pytest.mark.parametrize('bar_x', [0.38, 0.39, 0.40, 0.41, 0.4175])
def test_trace_ends_with_t_at_a_t_bar(bar_x):
    m = painted([[-0.1, 0.0], [bar_x, 0.0]], [[bar_x, -0.15], [bar_x, 0.15]])
    pts, end = trace(m)
    assert end == 't'
    # the end point (later the node's t_point) is the last step before the bar: <= 2 cm short
    assert bar_x - 0.02 <= pts[-1][0] <= bar_x + 0.005
    assert abs(pts[-1][1]) < 0.005


def test_short_bar_is_not_a_t():
    """+-3 cm arms look like a blurred sharp corner, not the finish bar."""
    m = painted([[-0.1, 0.0], [0.4, 0.0]], [[0.4, -0.03], [0.4, 0.03]])
    _, end = trace(m)
    assert end != 't'


def test_crossing_goes_straight_through():
    m = painted([[-0.1, 0.0], [1.5, 0.0]], [[0.4, -0.3], [0.4, 0.3]])
    pts, end = trace(m)
    assert end == 'len'
    assert np.abs(pts[:, 1]).max() < 0.005
    assert pts[-1] == pytest.approx([0.8, 0.0], abs=0.005)


def test_side_branch_is_ignored():
    m = painted([[-0.1, 0.0], [1.5, 0.0]], [[0.4, 0.0], [0.4, 0.15]])
    pts, end = trace(m)
    assert end == 'len'
    assert np.abs(pts[:, 1]).max() < 0.01
    assert pts[-1][0] == pytest.approx(0.8, abs=0.005)


def test_one_sided_stub_is_not_a_t():
    """Line ending in a bar to one side only (an L): followed into the stub, then 'open'."""
    m = painted([[-0.1, 0.0], [0.4, 0.0], [0.4, 0.15]])
    pts, end = trace(m)
    assert end == 'open'
    assert pts[-1] == pytest.approx([0.4, 0.14], abs=0.015)


def test_dead_end_is_open():
    m = painted([[-0.1, 0.0], [0.4, 0.0]])
    pts, end = trace(m)
    assert end == 'open'
    assert 0.37 <= pts[-1][0] <= 0.41


def test_empty_map_is_open_immediately():
    pts, end = trace(LineMap())
    assert end == 'open' and len(pts) == 1


def _gap_trace(gap, x_end):
    sep = gap + 2 * synth.HALF_WIDTH              # round caps: visible gap = sep - 2 cm
    m = painted([[-0.1, 0.0], [x_end, 0.0]], [[x_end + sep, 0.0], [2.0, 0.0]])
    return trace(m)


@pytest.mark.parametrize('gap', [0.03, 0.05, 0.07])
def test_short_gaps_are_bridged(gap):
    for x_end in (0.30, 0.31, 0.32, 0.33):        # every phase w.r.t. the 2 cm trace step
        pts, end = _gap_trace(gap, x_end)
        assert end == 'len', f'gap {gap} not bridged at x_end={x_end}'
        assert pts[-1][0] > x_end + gap + 0.4
        assert np.abs(pts[:, 1]).max() < 0.005


@pytest.mark.parametrize('gap', [0.08, 0.09, 0.15])
def test_long_gaps_are_not_bridged(gap):
    for x_end in (0.30, 0.31, 0.32, 0.33):
        pts, end = _gap_trace(gap, x_end)
        assert end == 'open'
        assert pts[-1][0] <= x_end + 0.01


# ------------------------------------------------------------------ camera -> map -> trace
@pytest.mark.parametrize('bar_x', [0.55, 0.5575, 0.565])
def test_t_bar_seen_by_the_camera_is_traced_as_t(bar_x):
    from line_follower.perception.rule import RulePerception
    shapes = [np.array([[0.0, 0.0], [bar_x, 0.0]]), np.array([[bar_x, -0.15], [bar_x, 0.15]])]
    bgr, gray = synth.render(shapes)
    out = RulePerception(None).process(bgr, gray, 0.0, vision.X_FAR)
    m = LineMap()
    m.update(out.line, out.valid, (0.0, 0.0, 0.0))
    pts, end = trace(m, 0.36, 0.0, 0.0)
    assert end == 't'
    # camera map is ~3 mm farther than truth here (BEV skew), so allow a little beyond the bar
    assert bar_x - 0.025 <= pts[-1][0] <= bar_x + 0.006 and abs(pts[-1][1]) < 0.01


def test_crossing_seen_by_the_camera_is_not_a_t():
    from line_follower.perception.rule import RulePerception
    shapes = [np.array([[0.0, 0.0], [1.5, 0.0]]), np.array([[0.55, -0.3], [0.55, 0.3]])]
    bgr, gray = synth.render(shapes)
    out = RulePerception(None).process(bgr, gray, 0.0, vision.X_FAR)
    m = LineMap()
    m.update(out.line, out.valid, (0.0, 0.0, 0.0))
    pts, end = trace(m, 0.36, 0.0, 0.0)
    assert end == 'open'                       # runs straight on to the end of the view
    assert pts[-1][0] > 0.8 and np.abs(pts[:, 1]).max() < 0.01
