"""Bitwise regression against the pre-optimization update from 74b1b85."""
import math

import cv2
import numpy as np

from line_follower import vision
from line_follower.line_map import LineMap

RES = vision.RES
SIZE = 2400
C0 = SIZE // 2
PATCH = max(520, 2 * int(math.ceil(math.hypot(vision.X_FAR, vision.Y_HALF) / RES)) + 20)
_ADD_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))


class ReferenceLineMap:
    def __init__(self):
        self.grid = np.zeros((SIZE, SIZE), np.uint8)
        self.seen = np.zeros((SIZE, SIZE), np.uint8)     # cells the camera has looked at (restore.py)
        self.frozen = None          # cells the robot has already driven over: never rewritten (freeze())
        self.restored = None        # learned route where the camera has not looked (perception restores)
        # confidence of every line cell, handed to planning with the map (0-255, see confidence()):
        # near observations high, falling with distance; far-band cells lower; restored cells low
        self.conf = np.zeros((SIZE, SIZE), np.uint8)
        self.last = None

    def freeze(self, x, y, radius):
        """Mark the ground within radius [m] of the robot (odom) as final: later frames skip it."""
        if self.frozen is None:
            self.frozen = np.zeros((SIZE, SIZE), np.uint8)
        cv2.circle(self.frozen, (int(round(x / RES)) + C0, int(round(y / RES)) + C0),
                   int(round(radius / RES)), 255, -1)

    RESTORED_CONF = 80          # confidence reported for a restored (never seen) cell
    FAR_BAND_SCALE = 0.6        # far band (full-res strip near the horizon) vs near range

    @staticmethod
    def _bev_conf(far_mask=None):
        """Per-BEV-pixel confidence: 1.0 up to 0.45 m ahead, falling linearly to 0.35 at 1.6 m;
        far-band pixels scaled by FAR_BAND_SCALE."""
        x = vision.X_FAR - (np.arange(vision.BH) + 0.5) * RES
        row = np.clip(1.0 - (x - 0.45) / (1.6 - 0.45) * 0.65, 0.35, 1.0)
        c = np.repeat((row * 255)[:, None], vision.BW, axis=1)
        if far_mask is not None:
            c = np.where(far_mask > 0, c * ReferenceLineMap.FAR_BAND_SCALE, c)
        return c.astype(np.uint8)

    def update(self, bev_mask, bev_valid, pose, add_only=None, far_mask=None):
        """Overwrite the part of the map currently seen by the camera.

        add_only (BEV mask, optional): there the observation is less trusted. It never
        erases, and adds line only where the map has no line within ADD_GAP (fills what
        was never seen, e.g. a corner arc, without drawing a second, shifted copy of a
        known line).
        """
        px, py, th = pose
        ox = int(round(px / RES)) + C0 - PATCH // 2
        oy = int(round(py / RES)) + C0 - PATCH // 2
        if ox < 0 or oy < 0 or ox + PATCH > SIZE or oy + PATCH > SIZE:
            return
        c, s = math.cos(th), math.sin(th)
        # BEV pixel (col,row) -> robot (x,y) -> odom -> patch pixel
        ax, ay = vision.X_FAR, vision.Y_HALF
        tx = px / RES + C0 - ox
        ty = py / RES + C0 - oy
        m = np.float32([
            [s, -c, (c * ax - s * ay) / RES + tx],
            [-c, -s, (s * ax + c * ay) / RES + ty],
        ])
        obs = cv2.warpAffine(bev_mask, m, (PATCH, PATCH), flags=cv2.INTER_LINEAR)
        val = cv2.warpAffine(bev_valid, m, (PATCH, PATCH), flags=cv2.INTER_NEAREST)
        win = self.grid[oy:oy + PATCH, ox:ox + PATCH]
        cwin = self.conf[oy:oy + PATCH, ox:ox + PATCH]
        cobs = cv2.warpAffine(self._bev_conf(far_mask), m, (PATCH, PATCH), flags=cv2.INTER_NEAREST)
        seen = val > 0
        self.seen[oy:oy + PATCH, ox:ox + PATCH][seen] = 255
        if self.restored is not None:
            self.restored[oy:oy + PATCH, ox:ox + PATCH][seen] = 0      # seen now: the guess is void
        if self.frozen is not None:
            seen &= self.frozen[oy:oy + PATCH, ox:ox + PATCH] == 0     # the road already driven stays as it is
        if add_only is not None:
            add = cv2.warpAffine(add_only, m, (PATCH, PATCH), flags=cv2.INTER_NEAREST) > 0
            known = cv2.dilate((win > 0).astype(np.uint8), _ADD_KERNEL) > 0
            new = add & seen & (obs > 100) & ~known
            seen &= ~add
            win[seen] = np.where(obs[seen] > 100, 255, 0)
            win[new] = 255
            cwin[seen] = np.where(obs[seen] > 100, cobs[seen], 0)
            cwin[new] = cobs[new]
        else:
            win[seen] = np.where(obs[seen] > 100, 255, 0)
            cwin[seen] = np.where(obs[seen] > 100, cobs[seen], 0)
        self.last = (ox, oy, obs > 100, seen)       # for the debug image

def test_update_matches_frozen_reference():
    rng = np.random.default_rng(67129)
    actual, reference = LineMap(), ReferenceLineMap()
    shape = (vision.BH, vision.BW)
    frame = np.zeros((vision.H, vision.W), np.uint8)
    edge = (C0 - PATCH // 2) * RES
    for step in range(320):
        angle = step * 0.039
        pose = (1.3 * math.cos(angle), 1.3 * math.sin(angle),
                angle + math.pi / 2 + rng.normal(0, 0.23))
        if step % 32 == 0:
            pose = (edge + RES, -edge - RES, angle)
        elif step % 32 == 1:
            pose = (edge, -edge, angle)

        line = np.zeros(shape, np.uint8)
        control = rng.integers(0, vision.BW, size=(4, 2), dtype=np.int32)
        control[:, 1] = np.linspace(0, vision.BH - 1, 4, dtype=np.int32)
        cv2.polylines(line, [control], False, int(rng.integers(101, 256)), 5)
        _, valid = vision.bev_warp(frame, theta=rng.uniform(-0.18, 0.18))
        if step % 13 == 0:
            valid[:] = 0
        if step % 19 == 0:
            line[:] = 0
        far = valid.copy()
        far[vision.BH // 3:] = 0
        far_mask = far if step % 3 else None
        add_only = far if step % 4 else None
        if step % 17 == 0:
            add_only = np.full(shape, 1, np.uint8)
        elif step % 17 == 1:
            add_only = valid

        if step == 12:
            actual.restored = np.full((SIZE, SIZE), 255, np.uint8)
            reference.restored = actual.restored.copy()
        if step >= 7 and step % 3 == 1:
            for mapping in (actual, reference):
                mapping.freeze(*pose[:2], radius=0.24)
        previous = actual.last
        if previous is not None:
            previous_masks = tuple(mask.copy() for mask in previous[2:])

        for mapping in (actual, reference):
            mapping.update(line, valid, pose, add_only, far_mask)
        assert actual.__dict__.keys() == reference.__dict__.keys()
        for key in ("grid", "seen", "restored", "frozen", "conf"):
            left, right = getattr(actual, key), getattr(reference, key)
            if right is None:
                assert left is None, (step, key)
            else:
                assert left.dtype == right.dtype and np.array_equal(left, right), (step, key)
        if reference.last is None:
            assert actual.last is None
        else:
            assert actual.last[:2] == reference.last[:2]
            for left, right in zip(actual.last[2:], reference.last[2:]):
                assert left.dtype == right.dtype
                assert np.array_equal(left.view(np.uint8), right.view(np.uint8)), step
        if previous is not None:
            for left, right in zip(previous[2:], previous_masks):
                assert np.array_equal(left, right), step
