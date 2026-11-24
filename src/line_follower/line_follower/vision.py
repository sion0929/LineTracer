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


def bev_warp(img, theta=0.0, x_max=X_FAR):
    """Camera-image-sized array -> (BEV image, BEV validity mask 0/255).

    Shared by every perception plugin so they all write the same ground grid.
    theta is the nose-up pitch of the camera relative to the ground [rad].
    Only ground closer than x_max is reported as seen (far cells are the most
    sensitive to pitch errors).
    """
    src = np.float32([ground_to_image(x, y, theta) for x, y in _GP])
    m = cv2.getPerspectiveTransform(src, _DST)
    valid = cv2.warpPerspective(_ONES, m, (BW, BH), flags=cv2.INTER_NEAREST)
    valid = cv2.erode(valid, _ERODE, borderValue=0)
    if x_max < X_FAR:
        valid[:int((X_FAR - x_max) / RES), :] = 0
    bev = cv2.warpPerspective(img, m, (BW, BH), flags=cv2.INTER_LINEAR)
    return bev, valid


def line_mask(gray, theta=0.0, x_max=X_FAR, thresh_cap=125.0):
    """Gray camera image -> (BEV line mask, BEV validity mask), both 0/255 (rule plugin)."""
    bev, valid = bev_warp(gray, theta, x_max)
    seen = valid > 0
    if not seen.any():
        return np.zeros((BH, BW), np.uint8), valid
    # the line is black on a bright mat: anything much darker than the typical ground
    t = min(thresh_cap, 0.62 * float(np.median(bev[seen])))
    return ((bev < t) & seen).astype(np.uint8) * 255, valid
