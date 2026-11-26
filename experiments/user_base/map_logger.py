"""Shared truth-aligned map logging for the development followers."""
import math
import os
import time

import numpy as np

import gt
from line_follower import vision
from line_follower.line_map import C0, RES


class MapLogger:
    log_video = False
    save_empty_map = False

    def __init__(self):
        super().__init__()
        if not hasattr(self, "truth"):
            self.truth = gt.Truth()
        self._map_out = gt.job_dir() / "map_world.npz"
        self.cells = {}            # (i, j) world 5 mm cell -> (t, dist, travel, conf)
        self.rcells = {}           # restored-layer cells (perception's learned guess), same keys
        self.track = []
        self.t0 = time.monotonic()
        self.create_timer(0.5, self._log)
        self.create_timer(5.0, self._save)
        if self.log_video:
            self.vdir = gt.job_dir() / 'video'
            self.vdir.mkdir(exist_ok=True)
            self.vk = 0
            self.last_msg = None
            self.create_timer(0.2, self._frame)
        self.get_logger().info(f'map logger, X_FAR {vision.X_FAR} (dev only)')

    def _log(self):
        full = self.truth.get_full()
        if full is None or self.pose is None or not self.hist:
            return
        tx, ty, tth, _, _, stamp = full
        if abs(self.w_cmd) > 1.0 or stamp > self.hist[-1][0] or stamp < self.hist[0][0]:
            return                       # fast spin or no odometry around the truth stamp: skip
        px, py, pth = self.pose_at(stamp)   # node pose at the same sim time as the truth
        self.track.append((time.monotonic() - self.t0, tx, ty, tth, px, py, pth, self.travel))
        r = int((vision.X_FAR + 0.4) / RES)
        ci, cj = int(round(py / RES)) + C0, int(round(px / RES)) + C0
        i0, j0 = max(0, ci - r), max(0, cj - r)
        sub = self.map.grid[i0:ci + r, j0:cj + r]
        ii, jj = np.nonzero(sub > 60)
        rest = self.map.restored
        if rest is not None:
            ri, rj = np.nonzero(rest[i0:ci + r, j0:cj + r] > 0)
            if len(ri):
                c0, s0 = math.cos(pth), math.sin(pth)
                ox_, oy_ = (rj + j0 - C0) * RES - px, (ri + i0 - C0) * RES - py
                rx_, ry_ = ox_ * c0 + oy_ * s0, -ox_ * s0 + oy_ * c0
                c2_, s2_ = math.cos(tth), math.sin(tth)
                tnow = time.monotonic() - self.t0
                for a_, b_ in zip(np.round((tx + rx_ * c2_ - ry_ * s2_) / RES).astype(int),
                                  np.round((ty + rx_ * s2_ + ry_ * c2_) / RES).astype(int)):
                    self.rcells.setdefault((a_, b_), (tnow, self.travel))
        if not len(ii):
            return
        ox = (jj + j0 - C0) * RES
        oy = (ii + i0 - C0) * RES
        # odom -> robot (node pose) -> world (truth pose)
        dx, dy = ox - px, oy - py
        c, s = math.cos(pth), math.sin(pth)
        rx, ry = dx * c + dy * s, -dx * s + dy * c
        c2, s2 = math.cos(tth), math.sin(tth)
        wx, wy = tx + rx * c2 - ry * s2, ty + rx * s2 + ry * c2
        dist = np.hypot(rx, ry)
        t = time.monotonic() - self.t0
        for a, b, d, x, y in zip(np.round(wx / RES).astype(int), np.round(wy / RES).astype(int),
                                 dist, ox, oy):
            if (a, b) not in self.cells:
                self.cells[(a, b)] = (t, float(d), self.travel, self.map.confidence(x, y))

    def on_image(self, msg):
        if self.log_video:
            self.last_msg = msg
        super().on_image(msg)

    def _frame(self):
        import cv2
        msg = self.last_msg
        if msg is None or self.pose is None:
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        cam = cv2.cvtColor(img, cv2.COLOR_RGB2BGR).copy()
        # BEV footprint rows: near end / split / far end on the image (flat ground)
        gp = self.get_parameter
        split = gp('far_split').value if gp('far_band').value else vision.X_FAR
        for x, col in ((vision.X_NEAR, (0, 200, 255)), (split, (0, 200, 255)), (vision.X_FAR, (255, 120, 0))):
            v = int(round(vision.ground_to_image(x, 0.0)[1]))
            cv2.line(cam, (0, v), (cam.shape[1], v), col, 1)
        if gp('far_band').value:
            v0 = int(round(vision.ground_to_image(vision.X_FAR, 0.0)[1])); v1 = int(round(vision.ground_to_image(split, 0.0)[1]))
            ov = cam.copy(); cv2.rectangle(ov, (0, v0), (cam.shape[1], v1), (255, 120, 0), -1)
            cam = cv2.addWeighted(ov, 0.25, cam, 0.75, 0)
        # map window in odom: x, y in [-0.6, 3.4] m
        x0, y0, n = -0.6, -0.6, 800
        j0, i0 = int(round(x0 / RES)) + C0, int(round(y0 / RES)) + C0
        g = self.map.grid[i0:i0 + n, j0:j0 + n]
        s = self.map.seen[i0:i0 + n, j0:j0 + n]
        m = np.full((n, n, 3), 24, np.uint8)
        m[s > 0] = (58, 52, 46)
        m[g > 60] = (255, 255, 255)
        def px(X, Y):
            return int(round((X - x0) / RES)), int(round((Y - y0) / RES))
        if len(self.path) > 1:
            pts = np.array([px(*q) for q in self.path], np.int32)
            cv2.polylines(m, [pts], False, (80, 220, 80), 3)
        X, Y, th = self.pose
        c, sn = math.cos(th), math.sin(th)
        foot = [(vision.X_NEAR, vision.Y_HALF), (vision.X_FAR, vision.Y_HALF), (vision.X_FAR, -vision.Y_HALF), (vision.X_NEAR, -vision.Y_HALF)]
        poly = np.array([px(X + a * c - b * sn, Y + a * sn + b * c) for a, b in foot], np.int32)
        cv2.polylines(m, [poly], True, (0, 200, 255), 1)
        tri = np.array([px(X + a * c - b * sn, Y + a * sn + b * c) for a, b in ((0.12, 0), (-0.06, 0.07), (-0.06, -0.07))], np.int32)
        cv2.fillPoly(m, [tri], (60, 90, 255))
        m = cv2.flip(m, 0)                                   # +y up
        cv2.imwrite(str(self.vdir / f'cam_{self.vk:05d}.jpg'), cam, [cv2.IMWRITE_JPEG_QUALITY, 85])
        cv2.imwrite(str(self.vdir / f'map_{self.vk:05d}.png'), cv2.resize(m, (480, 480), interpolation=cv2.INTER_AREA))
        self.vk += 1

    def _save(self):
        if not self.cells and not self.save_empty_map:
            return
        k = np.array(list(self.cells.keys()), float).reshape(-1, 2) * RES
        v = np.array(list(self.cells.values()), float).reshape(-1, 4)
        tmp = str(self._map_out) + '.tmp.npz'
        np.savez_compressed(tmp, xy=k, t=v[:, 0], dist=v[:, 1], travel=v[:, 2],
                            conf=v[:, 3], track=np.array(self.track, float).reshape(-1, 8),
                            x_far=vision.X_FAR,
                            far_stats=np.array(getattr(self, 'far_stats', [0, 0])),
                            rxy=np.array(list(self.rcells.keys()), float).reshape(-1, 2) * RES,
                            rinfo=np.array(list(self.rcells.values()), float).reshape(-1, 2))
        os.replace(tmp, self._map_out)

    def destroy_node(self):
        try:
            self._save()
        finally:
            super().destroy_node()
