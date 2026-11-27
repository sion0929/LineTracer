"""Vision helpers: camera image -> bird's-eye-view (BEV) line mask on the ground plane.

Robot frame: x forward from the wheel axle, y to the left.
Camera: 76 mm ahead of the axle, 103 mm above the ground, looking straight ahead.
"""
import math
import os

import numpy as np
import cv2

CAM_X = 0.076
CAM_H = 0.103
HFOV = 1.085595
W, H = 640, 480
FX = (W / 2.0) / math.tan(HFOV / 2.0)
CX, CY = (W - 1) / 2.0, (H - 1) / 2.0

RES = 0.005                  # metres per BEV / map cell
X_NEAR = 0.31                # forward range seen by the camera (from the axle)
# LF_X_FAR (dev experiments): how far ahead the BEV reaches; the camera sees further
X_FAR = float(os.environ.get('LF_X_FAR', '0.85'))
Y_HALF = 0.55
BW = int(round(2 * Y_HALF / RES))
BH = int(round((X_FAR - X_NEAR) / RES))


def ground_to_image(x, y, theta=0.0):
    """Ground point in the robot frame -> pixel (u, v); camera pitched up by theta [rad]."""
    xc = x - CAM_X
    zc = xc * math.cos(theta) - CAM_H * math.sin(theta)
    return CX - FX * y / zc, CY + FX * (xc * math.sin(theta) + CAM_H * math.cos(theta)) / zc


_GP = [(X_FAR, Y_HALF), (X_FAR, -Y_HALF), (X_NEAR, -Y_HALF), (X_NEAR, Y_HALF)]
_DST = np.float32([[0, 0], [BW - 1, 0], [BW - 1, BH - 1], [0, BH - 1]])
_ONES = np.full((H, W), 255, np.uint8)
_ERODE = np.ones((7, 7), np.uint8)


def bev_to_robot(col, row):
    return X_FAR - row * RES, Y_HALF - col * RES


def bev_warp(img, theta=0.0, x_max=X_FAR, scale=1.0):
    """Camera-image-sized array -> (BEV image, BEV validity mask 0/255).

    Shared by every perception plugin so they all write the same ground grid.
    theta is the nose-up pitch of the camera relative to the ground [rad].
    Only ground closer than x_max is reported as seen (far cells are the most
    sensitive to pitch errors).
    scale: img is the camera image resized by this factor (0.5 = 320x240).
    """
    src = np.float32([ground_to_image(x, y, theta) for x, y in _GP]) * scale
    m = cv2.getPerspectiveTransform(src, _DST)
    ones = _ONES if scale == 1.0 else np.full(img.shape[:2], 255, np.uint8)
    valid = cv2.warpPerspective(ones, m, (BW, BH), flags=cv2.INTER_NEAREST)
    valid = cv2.erode(valid, _ERODE, borderValue=0)
    if x_max < X_FAR:
        valid[:int((X_FAR - x_max) / RES), :] = 0
    bev = cv2.warpPerspective(img, m, (BW, BH), flags=cv2.INTER_LINEAR)
    return bev, valid


def line_mask(gray, theta=0.0, x_max=X_FAR, thresh_cap=125.0, scale=1.0):
    """Gray camera image -> (BEV line mask, BEV validity mask), both 0/255 (rule plugin).
    scale < 1: work on the image downscaled by this factor (near range, low resolution)."""
    if scale != 1.0:
        gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    bev, valid = bev_warp(gray, theta, x_max, scale)
    seen = valid > 0
    if not seen.any():
        return np.zeros((BH, BW), np.uint8), valid
    # the line is black on a bright mat: anything much darker than the typical ground
    t = min(thresh_cap, 0.62 * float(np.median(bev[seen])))
    return ((bev < t) & seen).astype(np.uint8) * 255, valid


def remove_wide(mask, max_width_m):
    """Remove connected components of a BEV line mask thicker than max_width_m [m].

    The true line is 2 cm wide (4 cells at RES = 5 mm). For every connected component
    the distance transform gives its largest inscribed-circle radius r, so its thickness
    is 2*r*RES regardless of the direction the line runs. Components thicker than
    max_width_m are removed WHOLE rather than only their thick core: a shadow blob often
    has a ragged 1-2 cell rim that would otherwise survive as a false thin line, while a
    real line joined to a blob is already corrupted at that junction.
    max_width_m <= 0 disables the filter and returns the mask unchanged (bit-identical).
    """
    if max_width_m <= 0.0:
        return mask
    binary = (mask > 0).astype(np.uint8)
    n, labels, _, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if n <= 1:
        return mask.copy()
    dist = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
    out = mask.copy()
    limit = max_width_m / RES                    # remove when 2*r > max_width_m/RES
    for label in range(1, n):
        component = labels == label
        if 2.0 * float(dist[component].max()) > limit:
            out[component] = 0
    return out


def far_band_mask(gray, theta, x0, x1, margin_px=6, thresh_cap=125.0, line_img=None, thin=0.0):
    """Far range x0..x1 [m] from the FULL-resolution image (dual resolution).

    Far ground is a thin band of rows near the horizon (0.85-1.2 m = rows ~288-310 of 480);
    a line crossing the view there is ~1 px thick at 640x480 and vanishes at 320x240.
    Only that band is warped. Returns (BEV line, BEV valid) on the full BEV grid, 0/255,
    nonzero only for rows of x0..x1. line_img: a full-size 0/255 line image (learned detector,
    far_band.py) used instead of the threshold.
    """
    x1 = min(x1, X_FAR)
    line = np.zeros((BH, BW), np.uint8)
    valid = np.zeros((BH, BW), np.uint8)
    if x1 <= x0:
        return line, valid
    quad = [(x1, Y_HALF), (x1, -Y_HALF), (x0, -Y_HALF), (x0, Y_HALF)]
    src = np.float32([ground_to_image(x, y, theta) for x, y in quad])
    v0 = int(max(0, np.floor(min(src[0][1], src[1][1])) - margin_px))
    v1 = int(min(H, np.ceil(max(src[2][1], src[3][1])) + margin_px))
    if v1 - v0 < 2:
        return line, valid
    band = gray[v0:v1] if line_img is None else line_img[v0:v1]
    r0, r1 = int(round((X_FAR - x1) / RES)), min(BH, int(round((X_FAR - x0) / RES)))
    if r1 - r0 < 2:
        return line, valid
    dst = np.float32([[0, 0], [BW - 1, 0], [BW - 1, r1 - r0 - 1], [0, r1 - r0 - 1]])
    src[:, 1] -= v0
    m = cv2.getPerspectiveTransform(src, dst)
    vb = cv2.warpPerspective(np.full(band.shape, 255, np.uint8), m, (BW, r1 - r0), flags=cv2.INTER_NEAREST)
    vb = cv2.erode(vb, np.ones((3, 7), np.uint8), borderValue=0)
    bb = cv2.warpPerspective(band, m, (BW, r1 - r0), flags=cv2.INTER_LINEAR)
    seen = vb > 0
    if not seen.any():
        return line, valid
    if line_img is not None:
        band_line = (bb > 127) & seen
        if thin > 0:
            band_line = thin_across(band_line, int(round(thin / RES)))
        line[r0:r1] = band_line.astype(np.uint8) * 255
    else:
        # threshold from the near ground (bottom of the image): the far band can hold the mat
        # edge, the floor beyond it and the horizon, which would pull a band median down
        t = min(thresh_cap, 0.62 * float(np.median(gray[int(H * 0.75):])))
        line[r0:r1] = ((bb < t) & seen).astype(np.uint8) * 255
    valid[r0:r1] = vb
    return line, valid


def thin_across(mask, max_run, keep=1):
    """Far rows are coarse (one image row ~2 cm of ground), so a line crossing the view comes out
    several cm thick along x. In every BEV column, runs no longer than max_run cells are cut to
    their centre (+-keep); longer runs are lines running away from the robot and stay."""
    out = mask.copy()
    m = mask.astype(np.int8)
    pad = np.zeros((1, m.shape[1]), np.int8)
    d = np.diff(np.vstack([pad, m, pad]), axis=0)
    for c in np.nonzero(m.any(0))[0]:
        starts = np.nonzero(d[:, c] == 1)[0]
        ends = np.nonzero(d[:, c] == -1)[0]
        for s, e in zip(starts, ends):
            if e - s <= max_run:
                mid = (s + e - 1) // 2
                out[s:e, c] = False
                out[max(s, mid - keep):min(e, mid + keep + 1), c] = True
    return out
