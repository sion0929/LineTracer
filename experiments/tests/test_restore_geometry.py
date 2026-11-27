"""Offline geometry tests for restore.py (no rclpy, no ONNX model needed).

Run:
    /home/rvlab/miniforge3/envs/lf_b_dl/bin/python -m pytest experiments/tests/test_restore_geometry.py -q
"""
import math
import sys
from pathlib import Path

import numpy as np
import cv2

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "line_follower"))

from line_follower.line_map import LineMap, RES, C0          # noqa: E402
from line_follower.restore import Restorer, PRES, N, X1, Y0  # noqa: E402


def _draw_map(lm, pts_odom, thick=2):
    """Draw a polyline given in odom metres into the map grid (same style as make_dataset)."""
    p = np.round((np.asarray(pts_odom, float) / RES + C0) * 8).astype(np.int32)
    cv2.polylines(lm.grid, [p], False, 255, thickness=thick,
                  lineType=cv2.LINE_8, shift=3)


def _world_to_robot(X, Y, pose):
    """Odom point -> robot frame (x forward, y left), the convention make_dataset.to_px uses."""
    px, py, th = pose
    c, s = math.cos(th), math.sin(th)
    dx, dy = X - px, Y - py
    return dx * c + dy * s, -dx * s + dy * c


def _to_px(x, y):
    """Dataset convention: col = (0.8 - y) / 0.0125, row = (1.2 - x) / 0.0125."""
    return (Y0 - y) / PRES, (X1 - x) / PRES


def _blank_restorer():
    r = Restorer.__new__(Restorer)          # bypass ONNX loading: patch() needs no model
    r.sess, r.net, r.backend, r.calls = None, None, "test", 0
    return r


def _line_points(x0, y0, x1, y1, n=401):
    t = np.linspace(0.0, 1.0, n)
    return [(x0 + t * (x1 - x0), y0 + t * (y1 - y0)) for t in t]


def test_patch_places_map_line_at_dataset_pixel():
    straight = LineMap()
    _draw_map(straight, _line_points(-2.0, 0.35, 2.0, 0.35))

    lshape = LineMap()
    _draw_map(lshape, _line_points(-1.0, -0.4, 0.5, -0.4) +
                       _line_points(0.5, -0.4, 0.5, 1.0))

    restorer = _blank_restorer()
    poses = [
        (0.0, 0.0, 0.0),
        (0.3, 0.2, math.radians(30.0)),
        (0.2, 0.2, math.radians(135.0)),
        (0.2, -0.3, math.radians(-60.0)),
    ]
    checked_total = 0
    for name, lm, pts in [("straight", straight, _line_points(-2.0, 0.35, 2.0, 0.35, 801)),
                          ("L", lshape, _line_points(-1.0, -0.4, 0.5, -0.4, 401) +
                                               _line_points(0.5, -0.4, 0.5, 1.0, 401))]:
        for pose in poses:
            line, _seen = restorer.patch(lm, pose)
            assert line.shape == (N, N)
            checked = 0
            for X, Y in pts:
                x, y = _world_to_robot(X, Y, pose)
                # keep a 3 px guard band around the patch edge
                if not (-0.36 <= x <= 1.16 and -0.76 <= y <= 0.76):
                    continue
                col, row = _to_px(x, y)
                ci, ri = int(round(col)), int(round(row))
                if not (2 <= ci < N - 2 and 2 <= ri < N - 2):
                    continue
                # tolerance 2 px: the nearest lit patch pixel must be within the 5x5 box
                near = line[ri - 2:ri + 3, ci - 2:ci + 3]
                assert float(near.max()) > 0.5, (
                    f"{name} line missing at pose={pose}, odom=({X:.3f},{Y:.3f}), "
                    f"robot=({x:.3f},{y:.3f}), px=(col={col:.2f},row={row:.2f})")
                checked += 1
            assert checked >= 5, f"{name} line visible in only {checked} samples at pose {pose}"
            checked_total += checked
    assert checked_total > 50


# --------------------------------------------------------------------------- (b)
def _to_px_pts(pts_robot):
    a = np.asarray(pts_robot, float)
    return np.stack([(Y0 - a[:, 1]) / PRES, (X1 - a[:, 0]) / PRES], -1)


def _draw_prob(paths, thick=3):
    """paths: list of robot-frame polylines -> 128x128 probability map."""
    prob = np.zeros((N, N), np.float32)
    for pts in paths:
        p = np.round(_to_px_pts(pts) * 8).astype(np.int32)
        cv2.polylines(prob, [p], False, 1.0, thickness=thick,
                      lineType=cv2.LINE_8, shift=3)
    return prob


def _check_walk(prob, start, known, length, tip, tip_tol=0.05):
    out = _blank_restorer().extend_ridge(prob, start, known, length=length)
    assert len(out) >= 3, f"walk too short: {len(out)} points"
    for x, y in out:
        # recover the exact chain pixel: x = X1 - (r + 0.5) * PRES
        r = int(round((X1 - x) / PRES - 0.5))
        c = int(round((Y0 - y) / PRES - 0.5))
        assert prob[r, c] > 0.6, f"walk left the ridge at ({x:.3f},{y:.3f}) px=({c},{r})"
    ex, ey = tip
    lx, ly = out[-1]
    assert math.hypot(lx - ex, ly - ey) <= tip_tol, (
        f"walk stopped at ({lx:.3f},{ly:.3f}), far end is ({ex:.3f},{ey:.3f})")
    return out


def test_extend_ridge_straight():
    path = [(x, 0.0) for x in np.linspace(0.0, 1.0, 101)]
    _check_walk(_draw_prob([path]), (0.0, 0.0), [], length=2.0, tip=(1.0, 0.0))


def test_extend_ridge_90_degree_turn():
    path = ([(x, 0.0) for x in np.linspace(0.0, 0.4, 81)] +
            [(0.4, y) for y in np.linspace(0.0, 0.35, 71)])
    _check_walk(_draw_prob([path]), (0.0, 0.0), [], length=2.0, tip=(0.4, 0.35))


def test_extend_ridge_150_degree_acute_turn():
    a = math.radians(150.0)
    path = ([(x, 0.0) for x in np.linspace(0.0, 0.2, 41)] +
            [(0.2 + t * math.cos(a), t * math.sin(a)) for t in np.linspace(0.0, 0.5, 101)])
    tip = (0.2 + 0.5 * math.cos(a), 0.5 * math.sin(a))
    _check_walk(_draw_prob([path]), (0.0, 0.0), [], length=2.0, tip=tip)


def test_extend_ridge_u_turn():
    arc = [(0.35 + 0.15 * math.cos(t), 0.15 + 0.15 * math.sin(t))
           for t in np.linspace(-math.pi / 2, math.pi / 2, 81)]
    path = ([(x, 0.0) for x in np.linspace(0.0, 0.35, 71)] + arc +
            [(x, 0.3) for x in np.linspace(0.35, 0.1, 51)])
    _check_walk(_draw_prob([path]), (0.0, 0.0), [], length=2.0, tip=(0.1, 0.3))


def test_extend_ridge_known_path_blocks_walking_back():
    # The probability ridge continues both ahead and behind the robot.  The map-known
    # path is ordered oldest -> newest (follower_node passes self.path, whose last point
    # is the current frontier), so it lies behind the start.
    path = [(x, 0.0) for x in np.linspace(-0.38, 0.12, 121)]
    prob = _draw_prob([path])
    known = [(-t, 0.0) for t in np.linspace(0.38, 0.0, 30)]

    # Without the block the walk follows the farther (backward) end of the ridge.
    free = _blank_restorer().extend_ridge(prob, (0.0, 0.0), [], length=1.0)
    assert free and free[-1][0] < -0.3, f"expected free walk to go back, got {free[-1:]}"

    # With the known path the block must stop the backward walk and send it forward.
    out = _blank_restorer().extend_ridge(prob, (0.0, 0.0), known, length=1.0)
    assert len(out) >= 3
    assert max(x for x, _ in out) > 0.08, f"blocked walk did not go forward: {out[-3:]}"
    assert min(x for x, _ in out) > -0.05, f"blocked walk went back down the known line: {out}"
    for x, y in out:
        r = int(round((X1 - x) / PRES - 0.5))
        c = int(round((Y0 - y) / PRES - 0.5))
        assert prob[r, c] > 0.6
