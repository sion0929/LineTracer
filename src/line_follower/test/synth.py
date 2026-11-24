"""Synthetic camera frames for the line follower unit tests (no Gazebo, no ROS graph).

Renders what the Waffle Pi camera would see of black 2 cm tape lines on a beige mat,
using exactly the camera model of `line_follower.vision.ground_to_image`, run
backwards: every pixel centre (u, v) below the horizon is turned into a ray, the ray is
intersected with the ground plane z = 0, and the pixel is painted black when that
ground point lies within `half_width` (default 1 cm) of one of the polylines.

Conventions (identical to vision.py):
    robot frame   x forward from the wheel axle, y to the left, z up, metres
    camera        at (CAM_X, 0, CAM_H), optical axis pitched nose-up by `pitch` [rad]
    pixel (u, v)  integer index = pixel centre, principal point (CX, CY), focal FX

Optional dark boxes are real 3D axis-aligned boxes standing on the ground
(x0, x1, y0, y1, height), drawn with a ray/box slab test so they occlude the mat
like an obstacle would.

The helpers return plain numpy arrays so the tests can compute IoU / lateral error
against `truth_bev`, the ground-truth line mask in vision's BEV grid.
"""
import functools
import math

import cv2
import numpy as np

from line_follower import vision

MAT_BGR = (170, 188, 200)      # beige mat
LINE_BGR = (30, 30, 30)        # black tape
BOX_BGR = (45, 48, 52)         # dark obstacle
SKY_BGR = (120, 120, 120)      # above the horizon (never reaches the BEV)
HALF_WIDTH = 0.01              # 2 cm wide line
_EPS = 1e-9                    # 'within' is inclusive, robust to float rounding on cell centres
RENDER_X_MAX = 2.5             # ground further than this is plain mat (outside every BEV)


# ---------------------------------------------------------------- camera model
def pixel_ray(u, v, theta=0.0):
    """Pixel centre(s) -> ray direction (rx, ry, rz) in the robot frame (not normalised).

    Inverse of vision.ground_to_image: optical axis d = (cos t, 0, sin t), image right
    = -y, image down = (sin t, 0, -cos t).
    """
    a = (np.asarray(u, float) - vision.CX) / vision.FX
    b = (np.asarray(v, float) - vision.CY) / vision.FX
    ct, st = math.cos(theta), math.sin(theta)
    return ct + b * st, -a, st - b * ct


def pixel_to_ground(u, v, theta=0.0):
    """Pixel centre(s) -> ground point (x, y) in the robot frame; NaN above the horizon."""
    rx, ry, rz = pixel_ray(u, v, theta)
    with np.errstate(divide='ignore', invalid='ignore'):
        t = np.where(rz < 0.0, -vision.CAM_H / rz, np.nan)
    return vision.CAM_X + t * rx, t * ry


@functools.lru_cache(maxsize=8)
def _ground_samples(theta, ss):
    """Ground (x, y) and ray directions for ss x ss sub-samples of every pixel."""
    offs = (np.arange(ss) + 0.5) / ss - 0.5
    us = (np.arange(vision.W)[None, :, None, None] + offs[None, None, None, :])
    vs = (np.arange(vision.H)[:, None, None, None] + offs[None, None, :, None])
    shape = (vision.H, vision.W, ss, ss)
    us, vs = np.broadcast_to(us, shape), np.broadcast_to(vs, shape)
    rays = pixel_ray(us, vs, theta)
    gx, gy = pixel_to_ground(us, vs, theta)
    for arr in (gx, gy):
        arr.setflags(write=False)
    return gx, gy, tuple(np.ascontiguousarray(r) for r in rays)


# ---------------------------------------------------------------- geometry
def _as_polylines(polylines):
    return [np.asarray(p, float).reshape(-1, 2) for p in polylines]


def dist_to_polylines(x, y, polylines):
    """Euclidean distance from points (x, y) (any equal shapes) to the nearest polyline."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    best = np.full(x.shape, np.inf)
    for pl in _as_polylines(polylines):
        if len(pl) == 1:
            best = np.minimum(best, np.hypot(x - pl[0, 0], y - pl[0, 1]))
            continue
        for (ax, ay), (bx, by) in zip(pl[:-1], pl[1:]):
            ux, uy = bx - ax, by - ay
            ll = ux * ux + uy * uy
            if ll == 0.0:
                d = np.hypot(x - ax, y - ay)
            else:
                t = np.clip(((x - ax) * ux + (y - ay) * uy) / ll, 0.0, 1.0)
                d = np.hypot(x - (ax + t * ux), y - (ay + t * uy))
            best = np.minimum(best, d)
    return best


def _near_line(x, y, polylines, half_width):
    """Boolean mask: ground point within half_width of a polyline (NaN -> False)."""
    out = np.zeros(x.shape, bool)
    ok = np.isfinite(x) & (x < RENDER_X_MAX)
    if not ok.any():
        return out
    pls = _as_polylines(polylines)
    if not pls:
        return out
    allp = np.concatenate(pls)
    lo = allp.min(axis=0) - half_width
    hi = allp.max(axis=0) + half_width
    ok &= (x >= lo[0]) & (x <= hi[0]) & (y >= lo[1]) & (y <= hi[1])
    idx = np.nonzero(ok.ravel())[0]
    if idx.size:
        xs, ys = x.ravel()[idx], y.ravel()[idx]
        out.ravel()[idx] = dist_to_polylines(xs, ys, pls) <= half_width + _EPS
    return out


def _box_hits(rays, boxes):
    """Boolean mask: the ray from the camera hits one of the boxes (x0, x1, y0, y1, h)."""
    rx, ry, rz = rays
    origin = (vision.CAM_X, 0.0, vision.CAM_H)
    hit = np.zeros(rx.shape, bool)
    for x0, x1, y0, y1, hgt in boxes:
        t_in = np.full(rx.shape, -np.inf)
        t_out = np.full(rx.shape, np.inf)
        for r, o, lo, hi in ((rx, origin[0], x0, x1), (ry, origin[1], y0, y1), (rz, origin[2], 0.0, hgt)):
            with np.errstate(divide='ignore', invalid='ignore'):
                ta = (lo - o) / r
                tb = (hi - o) / r
            par = r == 0.0                      # ray parallel to this slab
            inside = (lo <= o) & (o <= hi)
            ta = np.where(par, -np.inf if inside else np.inf, ta)
            tb = np.where(par, np.inf if inside else -np.inf, tb)
            t_in = np.maximum(t_in, np.minimum(ta, tb))
            t_out = np.minimum(t_out, np.maximum(ta, tb))
        hit |= (t_in <= t_out) & (t_out > 0.0)
    return hit


# ---------------------------------------------------------------- rendering
def render(polylines=(), boxes=(), pitch=0.0, half_width=HALF_WIDTH, gain=1.0,
           noise=2.0, seed=0, ss=2, mat=MAT_BGR, line=LINE_BGR, box=BOX_BGR):
    """Render one 640x480 camera frame.

    polylines: iterable of (N, 2) robot-frame points [m]; boxes: iterable of
    (x0, x1, y0, y1, height) [m]; pitch: camera nose-up pitch [rad] (same sign as
    vision.ground_to_image's theta); gain: global brightness factor; noise: std of
    seeded Gaussian pixel noise; ss: ss x ss supersampling (anti-aliased edges).
    Returns (bgr uint8 HxWx3, gray uint8 HxW).
    """
    gx, gy, rays = _ground_samples(float(pitch), int(ss))
    ground = np.isfinite(gx)
    is_line = _near_line(gx, gy, polylines, half_width)
    is_box = _box_hits(rays, boxes) if boxes else np.zeros(gx.shape, bool)
    colour = np.empty(gx.shape + (3,), np.float64)
    colour[:] = SKY_BGR
    colour[ground] = mat
    colour[is_line] = line
    colour[is_box] = box
    img = colour.mean(axis=(2, 3)) * gain
    if noise > 0:
        img += np.random.default_rng(seed).normal(0.0, noise, img.shape)
    bgr = np.clip(np.rint(img), 0, 255).astype(np.uint8)
    return bgr, cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)


def truth_bev(polylines, half_width=HALF_WIDTH):
    """Ground-truth line mask in vision's BEV grid (uint8 0/255, shape (BH, BW)).

    A cell is line when its centre, mapped with vision.bev_to_robot, lies within
    half_width of a polyline.
    """
    rows, cols = np.mgrid[0:vision.BH, 0:vision.BW]
    x, y = vision.bev_to_robot(cols.astype(float), rows.astype(float))
    return (dist_to_polylines(x, y, polylines) <= half_width + _EPS).astype(np.uint8) * 255


# ---------------------------------------------------------------- shapes
def straight(y0=0.0, angle_deg=0.0, x0=0.0, x1=2.0, x_ref=None):
    """Straight line through (x_ref or x0, y0) with heading angle_deg, from x0 to x1."""
    xr = x0 if x_ref is None else x_ref
    k = math.tan(math.radians(angle_deg))
    return np.array([[x0, y0 + k * (x0 - xr)], [x1, y0 + k * (x1 - xr)]])


def arc(radius, length, start=(0.0, 0.0), heading=0.0, n=200):
    """Circular arc; radius > 0 turns left, < 0 turns right."""
    s = np.linspace(0.0, length, n)
    r = radius
    ph = s / r
    x = start[0] + r * (np.sin(heading + ph) - math.sin(heading))
    y = start[1] - r * (np.cos(heading + ph) - math.cos(heading))
    return np.stack([x, y], axis=1)


def lateral_at(polyline, x):
    """y where the polyline first crosses the forward distance x (None if it does not)."""
    pl = np.asarray(polyline, float)
    for (ax, ay), (bx, by) in zip(pl[:-1], pl[1:]):
        if (ax - x) * (bx - x) <= 0.0 and ax != bx:
            return ay + (x - ax) * (by - ay) / (bx - ax)
    return None


def to_grid_painter(line_map_module):
    """Return paint(grid, odom_polylines, half_width) that writes lines straight into a
    LineMap grid (odom frame) with the same 'within half_width' rule as render()."""
    res, c0, size = line_map_module.RES, line_map_module.C0, line_map_module.SIZE

    def paint(grid, polylines, half_width=HALF_WIDTH, value=255):
        pls = _as_polylines(polylines)
        allp = np.concatenate(pls)
        pad = half_width + 2 * res
        j0 = max(0, int(math.floor((allp[:, 0].min() - pad) / res)) + c0)
        j1 = min(size, int(math.ceil((allp[:, 0].max() + pad) / res)) + c0 + 1)
        i0 = max(0, int(math.floor((allp[:, 1].min() - pad) / res)) + c0)
        i1 = min(size, int(math.ceil((allp[:, 1].max() + pad) / res)) + c0 + 1)
        ii, jj = np.mgrid[i0:i1, j0:j1]
        X, Y = (jj - c0) * res, (ii - c0) * res
        win = grid[i0:i1, j0:j1]
        win[dist_to_polylines(X, Y, pls) <= half_width + _EPS] = value
        return grid

    return paint
