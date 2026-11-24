"""Line map in the odom frame + line tracer.

Every camera frame is projected to the ground and written into a grid fixed to the
odom frame.  The camera cannot see the first 0.3 m in front of the robot, so the map
is what lets the robot follow a line that is already below / beside it.
"""
import math
import numpy as np
import cv2

from . import vision

RES = vision.RES
SIZE = 2400                  # 12 m x 12 m
C0 = SIZE // 2
PATCH = 520                  # window written per frame (covers the whole camera view)

_ANGLES = np.deg2rad(np.arange(-100, 101, 4))
_CONE = np.deg2rad(np.arange(-32, 33, 4))


class LineMap:
    def __init__(self):
        self.grid = np.zeros((SIZE, SIZE), np.uint8)
        self.last = None

    def update(self, bev_mask, bev_valid, pose):
        """Overwrite the part of the map currently seen by the camera."""
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
        seen = val > 0
        win[seen] = np.where(obs[seen] > 100, 255, 0)
        self.last = (ox, oy, obs > 100, seen)       # for the debug image

    def hit(self, X, Y):
        i = int(round(Y / RES)) + C0
        j = int(round(X / RES)) + C0
        if 0 <= i < SIZE and 0 <= j < SIZE:
            return self.grid[i, j] > 0
        return False

    def snap(self, X, Y, radius=0.06):
        """Centre of the line nearest to (X, Y), or None."""
        r = int(radius / RES)
        i0 = int(round(Y / RES)) + C0
        j0 = int(round(X / RES)) + C0
        if i0 - r < 0 or j0 - r < 0 or i0 + r >= SIZE or j0 + r >= SIZE:
            return None
        win = self.grid[i0 - r:i0 + r + 1, j0 - r:j0 + r + 1]
        ii, jj = np.nonzero(win)
        if ii.size == 0:
            return None
        d2 = (ii - r) ** 2 + (jj - r) ** 2
        k = int(np.argmin(d2))
        if d2[k] > r * r:
            return None
        near = (np.abs(ii - ii[k]) <= 3) & (np.abs(jj - jj[k]) <= 3)
        return ((j0 - r + jj[near].mean() - C0) * RES, (i0 - r + ii[near].mean() - C0) * RES)

    def _clusters(self, X, Y, h, radius, span=_ANGLES):
        """Directions (relative to h) in which the line continues at `radius`."""
        flags = [self.hit(X + radius * math.cos(h + a), Y + radius * math.sin(h + a)) for a in span]
        out, s = [], None
        for i, f in enumerate(flags):
            if f and s is None:
                s = i
            if not f and s is not None:
                out.append(0.5 * (span[s] + span[i - 1]))
                s = None
        if s is not None:
            out.append(0.5 * (span[s] + span[-1]))
        return out

    def _bridge(self, X, Y, h):
        """The line stops: look a little further ahead for its continuation (map gaps)."""
        for r in (0.045, 0.06, 0.075, 0.09):
            c = self._clusters(X, Y, h, r, _CONE)
            if c:
                a = min(c, key=abs)
                return X + r * math.cos(h + a), Y + r * math.sin(h + a), h + 0.5 * a
        return None

    def trace(self, X, Y, h, length=0.8, step=0.02):
        """Follow the line from (X, Y) with heading h.

        Returns (points, end) where end is 'open' (map ends / line not seen yet),
        't' (line ends in a T bar: the finish) or 'len' (requested length reached).
        At a crossing the straightest branch is taken.
        """
        pts = [(X, Y)]
        end = 'len'
        for _ in range(int(length / step)):
            cands = self._clusters(X, Y, h, 0.03)
            if not cands:
                nxt = self._bridge(X, Y, h)
                if nxt is None:
                    end = 'open'
                    break
                X, Y, h = nxt
                pts.append((X, Y))
                continue
            fwd = [a for a in cands if abs(a) < math.radians(50)]
            side_l = [a for a in cands if a >= math.radians(50)]
            side_r = [a for a in cands if a <= -math.radians(50)]
            if not fwd and side_l and side_r:
                # bar to both sides and nothing ahead: a T if the bar is long on both sides
                # (a blurred sharp corner is not) and nothing continues further on
                far = self._clusters(X, Y, h, 0.06)
                if (any(a >= math.radians(50) for a in far) and any(a <= -math.radians(50) for a in far)
                        and self._bridge(X, Y, h) is None):
                    end = 't'
                    break
            a = min(cands, key=abs)
            h += 0.8 * a
            X += step * math.cos(h)
            Y += step * math.sin(h)
            c = self.snap(X, Y, 0.015)
            if c is not None:
                X, Y = 0.5 * (X + c[0]), 0.5 * (Y + c[1])
            pts.append((X, Y))
        return pts, end
