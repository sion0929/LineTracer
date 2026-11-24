"""Continuous scan-to-map matching: nudge the pose so fresh line pixels sit on the remembered map.

Dead reckoning drifts a few cm (wheel slip on ramps, pivots). After a turn the line seen now
and the line remembered from before the turn then lie side by side, the traced path loses its
anchor and the robot follows the wrong stroke. Each frame, before the frame is written into
the map, the BEV line pixels are matched against a distance field of the map around the
robot over a small window (+-4 cm, +-1.5 deg). A clear, well-constrained improvement moves the
pose (and its recent history) part of the way. Along a straight line only the sideways
component is observable, so only that is applied there.
"""
import math

import cv2
import numpy as np

from . import vision
from .line_map import C0, RES

_T = np.arange(-4, 5) * 0.01
_XY = np.array(np.meshgrid(_T, _T)).reshape(2, -1).T          # 81 translations
_ANG = np.deg2rad([-1.5, -0.75, 0.0, 0.75, 1.5])
SIGMA = 0.015


class ScanMatcher:
    def __init__(self, gain=0.5, margin=0.06, min_pts=40, radius=1.1):
        self.gain, self.margin, self.min_pts = gain, margin, min_pts
        self.r = int(round(radius / RES))
        self.applied = self.tried = 0

    def correct(self, node, line, valid, pose):
        """Return the corrected pose (and fix node.pose / node.hist), or None."""
        rows, cols = np.nonzero((line[::2, ::2] > 0) & (valid[::2, ::2] > 0))
        if len(rows) < self.min_pts:
            return None
        x, y = vision.bev_to_robot(cols * 2, rows * 2)
        pts = np.column_stack((x, y))
        if len(pts) > 220:
            pts = pts[np.linspace(0, len(pts) - 1, 220).astype(int)]
        ix, iy = (int(round(p / RES)) + C0 for p in pose[:2])
        x0, y0 = ix - self.r, iy - self.r
        grid = node.map.grid
        if x0 < 0 or y0 < 0 or ix + self.r >= grid.shape[1] or iy + self.r >= grid.shape[0]:
            return None
        patch = grid[y0:iy + self.r + 1, x0:ix + self.r + 1]
        if not patch.any():
            return None
        dist = cv2.distanceTransform((patch == 0).astype(np.uint8), cv2.DIST_L2, 3) * RES
        origin = np.array([(x0 - C0) * RES, (y0 - C0) * RES])
        h, w = dist.shape
        self.tried += 1

        def score(xy, ang):
            yaw = pose[2] + ang
            c, s = math.cos(yaw), math.sin(yaw)
            world = pts @ np.array([[c, s], [-s, c]]) + np.array(pose[:2]) - origin
            p = np.rint((world[None] + xy[:, None]) / RES).astype(int)
            px, py = p[..., 0], p[..., 1]
            inside = (px >= 0) & (px < w) & (py >= 0) & (py < h)
            d = np.full(inside.shape, 0.2)
            d[inside] = dist[py[inside], px[inside]]
            return np.exp(-0.5 * (d / SIGMA) ** 2).mean(axis=1)

        nominal = float(score(np.zeros((1, 2)), 0.0)[0])
        best, best_xy, best_ang, table = -1.0, None, 0.0, {}
        for ang in _ANG:
            sc = score(_XY, ang)
            table[ang] = sc
            i = int(sc.argmax())
            if sc[i] > best:
                best, best_xy, best_ang = float(sc[i]), _XY[i], ang
        # only well-supported, clearly better matches (most pixels already on known line)
        if best < 0.45 or best - nominal < self.margin:
            return None
        # constrained directions: from the spread of the matched points
        cov = np.cov((pts @ np.array([[math.cos(pose[2]), math.sin(pose[2])],
                                       [-math.sin(pose[2]), math.cos(pose[2])]])).T)
        ev, evec = np.linalg.eigh(cov)
        dxy = np.array(best_xy, float)
        if ev[0] < 0.07 * ev[1]:                     # line-like: keep only the sideways part
            n = evec[:, 0]
            dxy = n * float(dxy @ n)
            best_ang = 0.0
        dxy *= self.gain
        dyaw = best_ang * self.gain
        if np.hypot(*dxy) < 0.003 and abs(dyaw) < math.radians(0.2):
            return None
        self.apply(node, pose, dxy[0], dxy[1], dyaw)
        self.applied += 1
        if self.applied % 20 == 1:
            node.get_logger().info(f'scan match {self.applied}/{self.tried}: {dxy[0]:+.3f} {dxy[1]:+.3f} m '
                                   f'{math.degrees(dyaw):+.2f} deg (score {nominal:.2f}->{best:.2f})')
        return True

    @staticmethod
    def apply(node, anchor, dx, dy, dyaw):
        ax, ay, _ = anchor
        c, s = math.cos(dyaw), math.sin(dyaw)

        def corrected(p):
            x, y, yaw = p
            return (ax + dx + c * (x - ax) - s * (y - ay),
                    ay + dy + s * (x - ax) + c * (y - ay),
                    math.remainder(yaw + dyaw, math.tau))
        node.pose = corrected(node.pose)
        node.hist[:] = [(t, corrected(p)) for t, p in node.hist]
        if getattr(node, 'yaw_off', None) is not None:
            node.yaw_off = math.remainder(node.yaw_off + dyaw, math.tau)
