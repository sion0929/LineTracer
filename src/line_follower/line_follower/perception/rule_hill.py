"""Rule perception + hill correction from the known line width (lead experiment).

Flat-ground IPM is exact on flat ground but badly wrong where the ground ahead is raised
or lowered (offline: 21.8 cm median xy error on hill points vs 1.2 cm from the line width,
docs/OPEN_SOURCE_SURVEY.md section 4). This plugin keeps the rule plugin's IPM output and,
for image rows whose line points are clearly off the robot's own ground plane, replaces
them with 3D points recovered from the 2 cm line width:

    Z = f * 0.02 / (w_px * cos(phi))        (forward depth along the camera axis)
    P = Z * ray(u, v)                       (camera frame, pitched by the IMU pitch)
    h = CAM_H + P_z                         (height above the robot's own ground)

Rows with |h| > hill_h are re-drawn at the width-based (x, y); the IPM content of those
rows is cleared so the map is not written twice. Only geometry, no learning.
"""
import math

import cv2
import numpy as np

from .. import vision
from .base import Perception, PerceptionOutput

LINE_W = 0.020


class RuleHillPerception(Perception):
    def __init__(self, node):
        super().__init__(node)
        node.declare_parameter('hill.height_threshold', 0.010)   # m off the own ground plane
        node.declare_parameter('hill.min_cos_phi', 0.6)
        node.declare_parameter('hill.enable', True)
        self.hill_h = node.get_parameter('hill.height_threshold').value
        self.min_cos = node.get_parameter('hill.min_cos_phi').value
        self.enable = node.get_parameter('hill.enable').value
        self.f = vision.FX
        # image row of every BEV row for a given pitch is recomputed per frame (cheap)
        self.bev_x = vision.X_FAR - (np.arange(vision.BH) + 0.5) * vision.RES
        self.last_hill_rows = 0

    # -------------------------------------------------------------- geometry
    def ipm_point(self, u, v, theta):
        """Flat-ground intersection of pixel (u, v): robot-frame (x from axle, y left)."""
        dx = 1.0
        dy = -(u - vision.CX) / self.f
        dz = -(v - vision.CY) / self.f
        # rotate camera ray by pitch theta (nose up)
        c, s = math.cos(theta), math.sin(theta)
        rx, rz = c * dx - s * dz, s * dx + c * dz
        if rz >= -1e-6:
            return None
        t = vision.CAM_H / -rz
        return vision.CAM_X + t * rx, t * dy

    def width_point(self, u, v, w_px, cos_phi, theta):
        Z = self.f * LINE_W / (w_px * cos_phi)            # forward depth in the camera frame
        dx, dy, dz = 1.0, -(u - vision.CX) / self.f, -(v - vision.CY) / self.f
        c, s = math.cos(theta), math.sin(theta)
        px, pz = Z * (c * dx - s * dz), Z * (s * dx + c * dz)
        return vision.CAM_X + px, Z * dy, vision.CAM_H + pz

    # -------------------------------------------------------------- main
    def process(self, bgr, gray, pitch, x_max):
        line, valid = vision.line_mask(gray, pitch, x_max)
        if not self.enable:
            return PerceptionOutput(line=line, valid=valid)
        bev, _ = vision.bev_warp(gray, pitch, vision.X_FAR)
        seen = valid > 0
        if not seen.any():
            return PerceptionOutput(line=line, valid=valid)
        t = min(125.0, 0.62 * float(np.median(bev[seen])))
        img = gray < t
        horizon = int(vision.CY - self.f * math.tan(pitch)) + 8
        v_far = int(round(vision.ground_to_image(vision.X_FAR, 0.0, pitch)[1]))
        v_near = int(round(vision.ground_to_image(vision.X_NEAR, 0.0, pitch)[1]))
        v0, v1 = max(horizon, v_far, 0), min(v_near, vision.H - 1)
        hill_pts, hill_rows = [], set()
        prev = []            # runs of the previous sampled row: (u, streak, pending points)
        for v in range(v1, v0 - 1, -3):                       # near -> far
            row = img[v]
            idx = np.flatnonzero(np.diff(np.concatenate(([0], row.view(np.uint8), [0]))))
            cur = []
            for a, b in zip(idx[::2], idx[1::2]):
                w = int(b - a)
                uc = (a + b - 1) / 2.0
                if w < 3:
                    continue
                match = min(prev, key=lambda r: abs(r[0] - uc)) if prev else None
                if match is None or abs(match[0] - uc) > 12:
                    cur.append((uc, 0, []))
                    continue
                p1, p0 = self.ipm_point(uc, v, pitch), self.ipm_point(match[0], v + 3, pitch)
                if p1 is None or p0 is None:
                    cur.append((uc, 0, []))
                    continue
                ddx, ddy = p1[0] - p0[0], p1[1] - p0[1]
                n = math.hypot(ddx, ddy)
                cos_phi = abs(ddx) / n if n > 1e-4 else 0.0
                if cos_phi < self.min_cos:
                    cur.append((uc, 0, []))
                    continue
                # width expected if this pixel were on the robot's own (flat) ground
                z_flat = (p1[0] - vision.CAM_X) * math.cos(pitch)   # ~ forward depth of the IPM point
                w_flat = self.f * LINE_W / (max(z_flat, 1e-3) * cos_phi)
                ratio = w / w_flat
                if ratio > 2.6:                                  # a box / blob, not a line
                    cur.append((uc, 0, []))
                    continue
                off = ratio > 1.25 or ratio < 0.8
                streak = match[1] + 1 if off else 0
                pend = (match[2] + [(uc, v, w, cos_phi)]) if off else []
                if streak >= 3:
                    for (uu, vv, ww, cc) in pend:
                        x, y, h = self.width_point(uu, vv, ww, cc, pitch)
                        if abs(h) > self.hill_h and vision.X_NEAR - 0.05 < x < vision.X_FAR + 0.6:
                            hill_pts.append((x, y))
                            hill_rows.add(vv)
                    pend = []
                cur.append((uc, streak, pend))
            prev = cur
        self.last_hill_rows = len(hill_rows)
        if not hill_rows:
            return PerceptionOutput(line=line, valid=valid)
        # clear BEV rows that come from hill image rows (their IPM distance is wrong)
        lo, hi = min(hill_rows) - 3, max(hill_rows) + 3
        line, valid = line.copy(), valid.copy()
        for r, xr in enumerate(self.bev_x):
            vr = vision.ground_to_image(xr, 0.0, pitch)[1]
            if lo <= vr <= hi:
                line[r, :] = 0
                valid[r, :] = 0
        # draw width-based points (they may lie beyond the IPM window: clip to the grid)
        for x, y in hill_pts:
            col = int(round((vision.Y_HALF - y) / vision.RES))
            row = int(round((vision.X_FAR - x) / vision.RES))
            if 0 <= row < vision.BH and 0 <= col < vision.BW:
                cv2.circle(valid, (col, row), 6, 255, -1)
                cv2.circle(line, (col, row), 2, 255, -1)
        return PerceptionOutput(line=line, valid=valid)
