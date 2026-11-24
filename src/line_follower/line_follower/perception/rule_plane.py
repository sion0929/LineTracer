"""Rule perception that picks the ground plane ahead from the line width (lead experiment).

On a slope the camera pitch relative to the ground ahead is ambiguous:
  * mid-slope, the ground ahead continues the slope  -> use the robot's rest pitch
  * at the foot of a ramp / on the crest, the ground ahead is level -> use the IMU pitch
The original node uses the rest pitch whenever |IMU pitch| > 3 deg, which is wrong at the
transitions (Gazebo: hard ramps make the robot leave the line at the foot of the descent).

Here each frame tries a few candidate camera-to-ground pitches and keeps the one whose
predicted line width (2 cm line, flat-plane IPM) best matches the measured pixel widths of
the thresholded line. Then the ordinary IPM with that pitch is used. Cheap and global.
"""
import math

import numpy as np

from .. import vision
from .base import Perception, PerceptionOutput

LINE_W = 0.020
REST_PITCH = math.radians(0.33)


class RulePlanePerception(Perception):
    def __init__(self, node):
        super().__init__(node)
        node.declare_parameter('plane.steps', 5)            # candidates between rest and IMU pitch
        node.declare_parameter('plane.slope_grid_deg', 0.0)  # >0: also try ground ahead tilted by +-this (hurt offline)
        node.declare_parameter('plane.slope_step_deg', 2.5)
        node.declare_parameter('plane.margin', 0.03)         # keep the default unless clearly better
        self.steps = int(node.get_parameter('plane.steps').value)
        self.grid = math.radians(node.get_parameter('plane.slope_grid_deg').value)
        self.gstep = math.radians(node.get_parameter('plane.slope_step_deg').value)
        self.margin = node.get_parameter('plane.margin').value
        self.f = vision.FX
        self.last_choice = None

    def candidates(self, pitch):
        imu = getattr(self.node, 'pitch_f', 0.0) + 0.0
        lo, hi = sorted((REST_PITCH, imu))
        cands = {round(pitch, 5), round(REST_PITCH, 5), round(imu, 5)}
        if hi - lo > math.radians(1.0):
            cands |= {round(lo + (hi - lo) * k / (self.steps - 1), 5) for k in range(self.steps)}
        if self.grid > 0:
            n = int(round(self.grid / self.gstep))
            cands |= {round(pitch + k * self.gstep, 5) for k in range(-n, n + 1)}
        return sorted(cands)

    def width_residual(self, img, theta, v_lo, v_hi):
        """median |log(measured / expected)| over line runs between image rows v_lo..v_hi."""
        errs = []
        for v in range(v_hi, v_lo - 1, -4):
            row = img[v]
            idx = np.flatnonzero(np.diff(np.concatenate(([0], row.view(np.uint8), [0]))))
            for a, b in zip(idx[::2], idx[1::2]):
                w = int(b - a)
                if w < 3 or w > 90:
                    continue
                # expected width on the plane: forward depth from the plane intersection
                dz = -(v - vision.CY) / self.f
                c, s = math.cos(theta), math.sin(theta)
                rz = s * 1.0 + c * dz
                if rz >= -1e-6:
                    continue
                t = vision.CAM_H / -rz
                depth = t                            # camera-axis depth of the ray hit (unit fwd)
                w_exp = self.f * LINE_W / depth      # lines running ~forward (cos phi ~ 1)
                errs.append(abs(math.log(w / w_exp)))
        return float(np.median(errs)) if len(errs) >= 6 else None

    def process(self, bgr, gray, pitch, x_max):
        cands = self.candidates(pitch)
        best = pitch
        if len(cands) > 1:
            # threshold as the rule plugin does (median of the seen ground)
            bev, valid = vision.bev_warp(gray, pitch, vision.X_FAR)
            seen = valid > 0
            if seen.any():
                t = min(125.0, 0.62 * float(np.median(bev[seen])))
                img = gray < t
                v_hi = min(int(round(vision.ground_to_image(vision.X_NEAR, 0.0, pitch)[1])), vision.H - 1)
                v_lo = max(int(round(vision.ground_to_image(0.75, 0.0, pitch)[1])), 0)
                scores = {th: self.width_residual(img, th, v_lo, v_hi) for th in cands}
                scores = {k: s for k, s in scores.items() if s is not None}
                if scores:
                    cand = min(scores, key=scores.get)
                    base = scores.get(round(pitch, 5))
                    # only leave the default plane when the width evidence is clearly better
                    if base is None or scores[cand] < base - self.margin:
                        best = cand
        self.last_choice = best
        line, valid = vision.line_mask(gray, best, x_max)
        return PerceptionOutput(line=line, valid=valid)
