"""Ground height ahead from the line width, and a BEV that follows that height (no flat-floor assumption).

The painted line is LINE_W wide. Where it runs roughly away from the camera, its width in an
image row gives the depth of that row (depth = FX * LINE_W / width_px), so every such row
yields a 3D point on the ground in the robot (body) frame without assuming a plane. A
piecewise-linear height profile z(x) (one optional bend: slope -> crest / foot) is fitted to
those points, and the BEV is rendered by intersecting each BEV cell's column with that
profile. Body rocking shows up as a tilted profile and is handled the same way.

Plain IPM assumes z = 0 (with an IMU pitch): 3 cm of height 1 m ahead moves the line ~29 cm.
"""
import math

import cv2
import numpy as np

from . import vision as V

LINE_W = 0.020
_XS = V.X_FAR - (np.arange(V.BH) + 0.5) * V.RES          # BEV row -> forward x [m]
_YS = V.Y_HALF - (np.arange(V.BW) + 0.5) * V.RES         # BEV col -> lateral y [m]


def _dark(gray, cap=125.0):
    lower = gray[V.H // 2:]
    t = min(cap, 0.62 * float(np.median(lower)))
    return gray < t


def width_points(gray, theta, v_min=None, max_slant=0.6):
    """3D ground points (x forward, y left, z up; robot frame) from line widths per image row.

    theta: camera pitch relative to the robot body (nose up) [rad].
    Returns array (N, 4): x, y, z, width_px.
    """
    dark = _dark(gray)
    if v_min is None:
        v_min = int(V.CY + 8)
    pts = []
    prev = []
    c, s = math.cos(theta), math.sin(theta)
    for v in range(V.H - 2, v_min, -2):
        row = dark[v]
        if not row.any():
            continue
        d = np.diff(np.r_[0, row.view(np.int8), 0])
        starts, ends = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
        cur = []
        for a, b in zip(starts, ends):
            w = b - a
            if w < 3 or w > 120 or a == 0 or b >= V.W:          # cut by the image border
                continue
            u = 0.5 * (a + b - 1)
            cur.append(u)
            # slant of the run from the previous processed row (2 px apart)
            near = [pu for pu in prev if abs(pu - u) < 12]
            if not near:
                continue
            slope = (u - min(near, key=lambda pu: abs(pu - u))) / 2.0       # du/dv
            if abs(slope) > max_slant:
                continue
            w_perp = w / math.sqrt(1.0 + slope * slope)
            depth = V.FX * LINE_W / w_perp
            yd = (v - V.CY) / V.FX * depth                      # down axis of the camera
            xc = depth * c + yd * s
            h = -depth * s + yd * c                             # height below the camera
            pts.append((xc + V.CAM_X, -(u - V.CX) / V.FX * depth, V.CAM_H - h, w))
        prev = cur
    return np.asarray(pts, float).reshape(-1, 4)


class Profile:
    """z(x) = z0 + s0 * x + ds * max(0, x - b)  (b = None: straight)."""

    def __init__(self, z0=0.0, s0=0.0, ds=0.0, b=None):
        self.z0, self.s0, self.ds, self.b = z0, s0, ds, b

    def __call__(self, x):
        z = self.z0 + self.s0 * x
        if self.b is not None:
            z = z + self.ds * np.maximum(0.0, x - self.b)
        return z


def fit_profile(p, min_pts=10, bend_gain=0.25, max_slope=0.30):
    """Robust fit of the height profile to points p (x, z). None if too few points."""
    if len(p) < min_pts:
        return None
    x, z = p[:, 0], p[:, 1]
    w = 1.0 / np.maximum(x, 0.3) ** 2                     # near rows are more precise

    def solve(cols):
        A = np.column_stack(cols)
        sw = np.sqrt(w)
        sol, *_ = np.linalg.lstsq(A * sw[:, None], z * sw, rcond=None)
        r = z - A @ sol
        return sol, float(np.sum(w * r * r) / np.sum(w)), r

    for _ in range(2):                                      # drop outliers (other lines, noise)
        sol, _, r = solve([np.ones(len(x)), x])
        mad = np.median(np.abs(r)) + 1e-4
        keep = np.abs(r) < 4 * 1.4826 * mad + 0.004
        x, z, w = x[keep], z[keep], w[keep]
        if len(x) < min_pts:
            return None
    (z0, s0), err1, _ = solve([np.ones(len(x)), x])
    best = Profile(z0, s0)
    best_err = err1
    if x.max() - x.min() > 0.25:
        for b in np.arange(max(0.35, x.min() + 0.08), x.max() - 0.08, 0.04):
            hinge = np.maximum(0.0, x - b)
            if (hinge > 0).sum() < 4 or (hinge == 0).sum() < 4:
                continue
            sol, e, _ = solve([np.ones(len(x)), x, hinge])
            if e < best_err * (1.0 - bend_gain):
                best, best_err = Profile(sol[0], sol[1], sol[2], b), e
    if abs(best.s0) > max_slope or abs(best.s0 + best.ds) > max_slope:
        return None
    return best


def ground_to_image_h(x, y, z, theta):
    """Like vision.ground_to_image for a point at height z above the robot's ground plane."""
    xc = x - V.CAM_X
    h = V.CAM_H - z
    zc = xc * math.cos(theta) - h * math.sin(theta)
    return V.CX - V.FX * y / zc, V.CY + V.FX * (xc * math.sin(theta) + h * math.cos(theta)) / zc


def bev_profile(gray, theta, prof, x_max=V.X_FAR):
    """BEV (same grid as vision.bev_warp) of the surface z = prof(x); returns (bev, valid)."""
    z = prof(_XS)
    c, s = math.cos(theta), math.sin(theta)
    xc = _XS - V.CAM_X
    h = V.CAM_H - z
    zc = xc * c - h * s                                     # (BH,)
    vv = V.CY + V.FX * (xc * s + h * c) / zc
    uu = V.CX - V.FX * _YS[None, :] / zc[:, None]
    map_x = uu.astype(np.float32)
    map_y = np.repeat(vv[:, None], V.BW, axis=1).astype(np.float32)
    bev = cv2.remap(gray, map_x, map_y, cv2.INTER_LINEAR, borderValue=0)
    valid = ((map_x >= 0) & (map_x <= V.W - 1) & (map_y >= 0) & (map_y <= V.H - 1)
             & (zc[:, None] > 0.05) & (_XS[:, None] <= x_max))
    return bev, (valid * 255).astype(np.uint8)


def line_mask_profile(gray, theta, prof, x_max=V.X_FAR, thresh_cap=125.0):
    bev, valid = bev_profile(gray, theta, prof, x_max)
    seen = valid > 0
    if not seen.any():
        return np.zeros((V.BH, V.BW), np.uint8), valid
    t = min(thresh_cap, 0.62 * float(np.median(bev[seen])))
    return ((bev < t) & seen).astype(np.uint8) * 255, valid


def calibrate(p, near_x=0.45, min_near=4):
    """Rescale depths so the near ground lies on the robot's plane (z = 0).

    The measured width depends on blur, exposure and the threshold, which scales every depth
    by the same factor. The robot stands on the ground just ahead, so the near points fix that
    factor; what remains is the shape of the terrain. Returns (points, scale) or (None, None).
    """
    near = p[:, 0] < near_x
    if near.sum() < min_near:
        return None, None
    drop = V.CAM_H - p[near, 2]
    ok = drop > 0.02
    if ok.sum() < min_near:
        return None, None
    sc = float(np.median(V.CAM_H / drop[ok]))
    if not 0.6 < sc < 1.6:
        return None, None
    q = p.copy()
    q[:, 0] = V.CAM_X + sc * (p[:, 0] - V.CAM_X)
    q[:, 1] = sc * p[:, 1]
    q[:, 2] = V.CAM_H - sc * (V.CAM_H - p[:, 2])
    return q, sc


def estimate(gray, theta_body, calib=True):
    """Height profile ahead, or None. theta_body: camera pitch relative to the body."""
    p = width_points(gray, theta_body)
    if len(p) == 0:
        return None, p
    if calib:
        p, _ = calibrate(p)
        if p is None:
            return None, np.zeros((0, 4))
    return fit_profile(p[:, [0, 2]]), p
