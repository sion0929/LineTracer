"""인지 (perception): camera model and bird's-eye-view projection in `vision.py`.

Covers ground_to_image / bev_to_robot / bev_warp / line_mask with the synthetic
renderer in `synth.py` (which is checked here first to be the exact inverse of
ground_to_image).  No ROS graph, no Gazebo.

Measured on rp/ros2:jazzy-lab (OpenCV 4.6), the numbers the thresholds below rest on:
  * robot -> image -> BEV -> robot round trip: worst 6.9 mm (<= 1 cell per axis); the
    BEV grid is skewed by up to one cell (see the KNOWN BUG xfail below)
  * straight lines at y = 0, +-5, +-15 cm: detected centre +2.5 mm off in every row
    where the whole line is inside the seen area (the half-cell skew)
  * validity mask: 11366 of 23760 cells, rows 3..104 (x = 0.835 .. 0.330 m)
  * a lateral bar at 0.60 m seen with an uncorrected 2 deg pitch lands at 0.52 m
    (nose up) / 0.72 m (nose down); with the right pitch at 0.6025 m
"""
import math

import cv2
import numpy as np
import pytest

import synth
from line_follower import vision

RES = vision.RES
BLANK = np.zeros((vision.H, vision.W), np.uint8)


def _row(x):
    return int(round((vision.X_FAR - x) / RES))


def _warp_matrix(theta=0.0):
    src = np.float32([vision.ground_to_image(x, y, theta) for x, y in vision._GP])
    return cv2.getPerspectiveTransform(src, vision._DST)


def _bar_x(gray, assumed_pitch):
    """Forward distance of a lateral bar seen in the middle BEV column."""
    line, _ = vision.line_mask(gray, assumed_pitch)
    rows = np.nonzero(line[:, vision.BW // 2] > 0)[0]
    assert rows.size, 'bar not detected'
    return vision.bev_to_robot(0, rows.mean())[0]


# ------------------------------------------------------------------ synthetic renderer
@pytest.mark.parametrize('pitch_deg', [0.0, 3.0, -3.0])
def test_synth_pixel_to_ground_is_exact_inverse_of_ground_to_image(pitch_deg):
    th = math.radians(pitch_deg)
    us, vs = np.meshgrid(np.arange(0, vision.W, 37), np.arange(300, vision.H, 23))
    x, y = synth.pixel_to_ground(us.ravel(), vs.ravel(), th)
    assert np.isfinite(x).all() and (x > vision.CAM_X).all()
    for xi, yi, u, v in zip(x, y, us.ravel(), vs.ravel()):
        bu, bv = vision.ground_to_image(xi, yi, th)
        assert bu == pytest.approx(u, abs=1e-6) and bv == pytest.approx(v, abs=1e-6)
    # and the other way round for ground points in the BEV area
    for gx, gy in [(0.31, 0.0), (0.5, 0.2), (0.85, -0.4), (0.62, 0.11)]:
        u, v = vision.ground_to_image(gx, gy, th)
        bx, by = synth.pixel_to_ground(u, v, th)
        assert float(bx) == pytest.approx(gx, abs=1e-9) and float(by) == pytest.approx(gy, abs=1e-9)


def test_synth_pixels_above_the_horizon_have_no_ground_point():
    x, _ = synth.pixel_to_ground(np.array([320.0, 320.0]), np.array([vision.CY - 1.0, 0.0]))
    assert np.isnan(x).all()


@pytest.mark.parametrize('pitch_deg', [0.0, 3.0])
def test_synth_paints_the_line_where_ground_to_image_projects_it(pitch_deg):
    th = math.radians(pitch_deg)
    k = math.tan(math.radians(10.0))
    _, gray = synth.render([synth.straight(0.05, 10.0, 0.0, 2.0, x_ref=0.5)], pitch=th, noise=0.0)
    checked = 0
    for x in np.linspace(0.32, 0.85, 30):
        y = 0.05 + k * (x - 0.5)
        u, v = (int(round(c)) for c in vision.ground_to_image(x, y, th))
        if not (0 <= u < vision.W and 0 <= v < vision.H):
            continue                          # nose up: the nearest ground leaves the image
        checked += 1
        assert gray[v, u] < 60, f'line missing at x={x:.3f}'
        for dy in (0.03, -0.03):              # 2 cm beside the line edge: plain mat
            u, v = (int(round(c)) for c in vision.ground_to_image(x, y + dy, th))
            assert gray[v, u] > 150, f'mat expected at x={x:.3f} dy={dy}'
    assert checked >= 25


def test_synth_colours_and_noise_are_deterministic():
    bgr1, gray1 = synth.render([synth.straight()])
    bgr2, _ = synth.render([synth.straight()])
    assert np.array_equal(bgr1, bgr2)
    mat = gray1[470, 20]                      # near ground, far left: plain mat
    assert 170 <= mat <= 200
    assert gray1[int(round(vision.ground_to_image(0.5, 0.0)[1])), 320] < 45


def test_synth_truth_bev_uses_bev_to_robot_convention():
    truth = synth.truth_bev([synth.straight(0.0)])
    assert truth.shape == (vision.BH, vision.BW)
    cols = np.nonzero(truth[_row(0.5)])[0]
    assert cols.tolist() == [108, 109, 110, 111, 112]   # |y| <= 1 cm, cell 110 is y = 0
    assert vision.bev_to_robot(110, 0)[1] == pytest.approx(0.0, abs=1e-12)


# ------------------------------------------------------------------ camera model
def test_grid_constants():
    assert (vision.BW, vision.BH) == (220, 108)
    assert vision.RES == 0.005
    assert vision.bev_to_robot(0, 0) == pytest.approx((vision.X_FAR, vision.Y_HALF))
    assert vision.bev_to_robot(110, 70) == pytest.approx((0.5, 0.0))


def test_ground_to_image_known_values():
    u, v = vision.ground_to_image(0.5, 0.0)
    assert u == pytest.approx(vision.CX)
    # a point on the left (y > 0) is left in the image (u < CX)
    assert vision.ground_to_image(0.5, 0.1)[0] < vision.CX < vision.ground_to_image(0.5, -0.1)[0]
    # farther ground is higher in the image and tends to the horizon row CY
    vs = [vision.ground_to_image(x, 0.0)[1] for x in (0.31, 0.5, 0.85, 1000.0)]
    assert vs == sorted(vs, reverse=True)
    assert vs[-1] == pytest.approx(vision.CY, abs=0.1)
    # X_NEAR is just inside the bottom of the image (v = 473.0 of 479)
    assert 465.0 < vs[0] < vision.H - 1
    # 1 px at 0.85 m is ~1.5 mm laterally: the 2 cm line is ~13.7 px wide there
    du = vision.ground_to_image(0.85, -0.01)[0] - vision.ground_to_image(0.85, 0.01)[0]
    assert du == pytest.approx(13.7, abs=0.1)


def test_bev_round_trip_within_one_cell():
    """robot -> ground_to_image -> bev_warp homography -> bev_to_robot."""
    m = _warp_matrix()
    worst = (0.0, 0.0)
    for row in range(0, vision.BH, 7):
        for col in range(0, vision.BW, 9):
            x, y = vision.bev_to_robot(col, row)
            u, v = vision.ground_to_image(x, y)
            c2, r2 = cv2.perspectiveTransform(np.float32([[[u, v]]]), m)[0, 0]
            xb, yb = vision.bev_to_robot(c2, r2)
            worst = (max(worst[0], abs(xb - x)), max(worst[1], abs(yb - y)))
    print(f'BEV round trip worst error: dx={worst[0] * 1000:.2f} mm, dy={worst[1] * 1000:.2f} mm')
    assert worst[0] <= RES + 1e-4 and worst[1] <= RES + 1e-4


@pytest.mark.xfail(strict=True, reason='KNOWN BUG: bev_warp maps the grid edges to cells BW-1/BH-1 '
                   'but bev_to_robot uses RES per cell, so BEV cells are skewed by up to 1 cell (5 mm)')
def test_bev_warp_scale_matches_bev_to_robot():
    m = _warp_matrix()
    x, y = vision.X_NEAR, -vision.Y_HALF + 0.05
    u, v = vision.ground_to_image(x, y)
    c, r = cv2.perspectiveTransform(np.float32([[[u, v]]]), m)[0, 0]
    xb, yb = vision.bev_to_robot(c, r)
    assert math.hypot(xb - x, yb - y) < 0.001


@pytest.mark.parametrize('offset', [0.0, 0.05, -0.05, 0.15, -0.15])
def test_line_at_known_lateral_offset_lands_within_two_cells(offset):
    _, gray = synth.render([synth.straight(offset)])
    line, valid = vision.line_mask(gray)
    errs = []
    for row in range(vision.BH):
        cols = np.nonzero(line[row] > 0)[0]
        # only rows where the whole line (+-1.5 cm) is inside the seen area
        edge = [int(round((vision.Y_HALF - (offset + d)) / RES)) for d in (0.015, -0.015)]
        if cols.size and valid[row, edge].all():
            assert cols.max() - cols.min() <= 6, f'row {row}: not one thin line'
            errs.append(vision.bev_to_robot(cols.mean(), row)[1] - offset)
    errs = np.array(errs)
    print(f'offset {offset:+.2f}: {errs.size} rows, mean err {errs.mean() * 1000:+.2f} mm, '
          f'worst {np.abs(errs).max() * 1000:.2f} mm')
    assert errs.size >= (95 if abs(offset) < 0.1 else 80)     # measured 102 / 88-89 rows
    assert np.abs(errs).max() <= 2 * RES + 1e-9
    assert abs(errs.mean()) <= RES


# ------------------------------------------------------------------ validity mask
def test_validity_mask_covers_exactly_the_visible_ground():
    _, valid = vision.bev_warp(BLANK)
    vb = valid > 0
    assert set(np.unique(valid)) <= {0, 255}
    rows, cols = np.mgrid[0:vision.BH, 0:vision.BW]
    x, y = vision.bev_to_robot(cols.astype(float), rows.astype(float))
    zc = x - vision.CAM_X
    u = vision.CX - vision.FX * y / zc
    v = vision.CY + vision.FX * vision.CAM_H / zc
    in_image = (u >= -0.5) & (u <= vision.W - 0.5) & (v >= -0.5) & (v <= vision.H - 0.5)
    # never claims ground outside the camera image ...
    assert not (vb & ~in_image).any()
    # ... and covers all of it except a band of a few cells (7x7 erosion) at the border
    core = cv2.erode(in_image.astype(np.uint8), np.ones((9, 9), np.uint8), borderValue=0) > 0
    assert not (core & ~vb).any()
    assert 11000 <= vb.sum() <= 11700            # measured 11366 of 23760 cells
    valid_rows = np.nonzero(vb.any(axis=1))[0]
    assert (valid_rows[0], valid_rows[-1]) == (3, 104)   # x = 0.835 .. 0.330 m
    c = np.nonzero(vb[_row(0.5)])[0]
    assert vision.bev_to_robot(c.min(), 0)[1] == pytest.approx(0.23, abs=0.011)
    assert vision.bev_to_robot(c.max(), 0)[1] == pytest.approx(-0.23, abs=0.011)


@pytest.mark.parametrize('x_max', [0.5, 0.6, 0.7])
def test_x_max_cuts_off_far_ground(x_max):
    _, full = vision.bev_warp(BLANK)
    _, valid = vision.bev_warp(BLANK, x_max=x_max)
    cut = int((vision.X_FAR - x_max) / RES)
    assert cut == _row(x_max)
    assert not valid[:cut].any()
    assert np.array_equal(valid[cut:], full[cut:])
    first = np.nonzero(valid.any(axis=1))[0][0]
    assert vision.bev_to_robot(0, first)[0] == pytest.approx(x_max, abs=1e-9)


def test_x_max_at_or_beyond_x_far_changes_nothing():
    _, full = vision.bev_warp(BLANK)
    _, same = vision.bev_warp(BLANK, x_max=vision.X_FAR + 0.2)
    assert np.array_equal(full, same)


def test_line_mask_drops_line_beyond_x_max():
    _, gray = synth.render([np.array([[0.75, -0.2], [0.75, 0.2]])])   # lateral bar at 0.75 m
    line, _ = vision.line_mask(gray)
    assert line[_row(0.75)].any()
    line, _ = vision.line_mask(gray, x_max=0.6)
    assert not line.any()


# ------------------------------------------------------------------ pitch
def test_nose_up_pitch_moves_ground_points_down_in_the_image():
    v0 = vision.ground_to_image(0.6, 0.0, 0.0)[1]
    up = vision.ground_to_image(0.6, 0.0, math.radians(2.0))[1]
    down = vision.ground_to_image(0.6, 0.0, math.radians(-2.0))[1]
    assert up > v0 > down
    assert up - v0 == pytest.approx(19.4, abs=0.2)
    assert v0 - down == pytest.approx(19.1, abs=0.2)


def test_uncorrected_pitch_shifts_the_bev_and_correction_removes_it():
    bar = [np.array([[0.6, -0.3], [0.6, 0.3]])]
    nose_up, nose_down = math.radians(2.0), math.radians(-2.0)
    _, g_up = synth.render(bar, pitch=nose_up)
    _, g_down = synth.render(bar, pitch=nose_down)
    _, g_flat = synth.render(bar)
    x_up, x_down = _bar_x(g_up, 0.0), _bar_x(g_down, 0.0)
    print(f'bar at 0.60 m: nose-up 2deg -> {x_up:.4f} m, nose-down 2deg -> {x_down:.4f} m (uncorrected)')
    assert x_up < 0.6 - 0.06          # measured 0.520: looks 8 cm nearer
    assert x_down > 0.6 + 0.10        # measured 0.7225: looks 12 cm farther
    for gray, th in ((g_up, nose_up), (g_down, nose_down), (g_flat, 0.0)):
        assert _bar_x(gray, th) == pytest.approx(0.6, abs=1.5 * RES)
