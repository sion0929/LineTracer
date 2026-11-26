"""Rule planner (team role 2): lane map from perception -> local path for control.

Reads the line map as perception hands it over (node.map: seen line + restored guess) and
builds the local path ahead: keep the path already committed (~8 cm lock), re-anchor it on the
map, trace forward (crossings straight through, gaps bridged), fall back to the remembered
path when the map loses the line, catch the finish T. Writes node.path (odom, ~2 cm),
node.path_end ('open' | 't' | 'len') and node.t_point.
"""
import math
import os
from pathlib import Path

import cv2
import numpy as np

from .. import vision
from .base import Planner, path_len


class RulePlanner(Planner):
    def prune(self):
        """Drop path points the axle has passed, measured along the path itself.

        A point is passed when the robot is nearer to the next point than to this one,
        seen along the segment between them (works for sharp corners and loops).
        """
        node = self.node
        px, py = node.pose[0], node.pose[1]
        n = 0
        while n + 1 < len(node.path) and n < 30:
            (ax, ay), (bx, by) = node.path[n], node.path[n + 1]
            ux, uy = bx - ax, by - ay
            if (px - ax) * ux + (py - ay) * uy <= 0.5 * (ux * ux + uy * uy):
                break
            n += 1
        if n:
            del node.path[:n]

    def acquire(self, x0=0.0):
        """Look for the line in front of the robot, starting x0 metres ahead of the axle."""
        node = self.node
        best = None
        for dx in np.arange(x0, 0.62, 0.02):
            for dy in (0.0, 0.03, -0.03, 0.06, -0.06, 0.10, -0.10, 0.15, -0.15, 0.2, -0.2):
                X, Y = node.to_odom(dx, dy)
                if node.map.hit(X, Y):
                    best = node.map.snap(X, Y, 0.03) or (X, Y)
                    break
            if best:
                break
        if best is None:
            return None, None
        bx, by = node.to_robot(*best)
        n = max(1, int(math.hypot(bx, by) / 0.02))
        link = [node.to_odom(bx * i / n, by * i / n) for i in range(1, n)]
        return link, best

    def replan(self):
        node = self.node
        if node.restorer is not None and node.get_parameter('restore_replace').value and node.travel >= 0.3 \
                and node.t_point is None and node.pose is not None:
            # the network sees the whole local map (double strokes from drift, misplaced
            # corners, gaps) and draws one route: follow that, anchored at the robot
            self.replan_map()                        # keeps the finish-T logic and map path as fallback
            prob = node.restorer.route(node.map, node.pose)
            gate = node.get_parameter('restore_gate').value
            if gate > 0.0 and len(node.path) >= 2:
                # does the map path agree with the learned route? (sampled 0.12-0.6 m ahead)
                rp = np.asarray([node.to_robot(*q) for q in node.path])
                sel = (rp[:, 0] > 0.12) & (rp[:, 0] < 0.6)
                if sel.sum() >= 5:
                    cols = np.clip(((0.8 - rp[sel, 1]) / 0.0125).astype(int), 0, 127)
                    rows = np.clip(((1.2 - rp[sel, 0]) / 0.0125).astype(int), 0, 127)
                    pk = cv2.dilate((prob * 255).astype(np.uint8), np.ones((3, 3), np.uint8))[rows, cols] / 255.0
                    long_enough = node.path_end != 'open' or path_len(node.path) > 0.45
                    if float(pk.mean()) >= gate and long_enough:
                        return                         # map path is confirmed: keep it
            rows, cols = np.nonzero(prob > 0.5)
            xs, ys = node.restorer.to_robot(cols, rows)
            near = (xs > 0.05) & (xs < 0.25) & (np.abs(ys) < 0.15)
            ext = []
            if near.any():
                k = int(np.argmin(np.hypot(xs[near] - 0.12, ys[near])))
                ext = node.restorer.extend(prob, (float(xs[near][k]), float(ys[near][k])), 0.0, 0.9)
            if len(ext) >= 10:
                node.path = [node.to_odom(x, y) for x, y in ext]
                node.path_end = 'open'
                node.n_restored = 0
            return
        # restored points are a fresh guess every frame, never remembered path
        n = getattr(node, 'n_restored', 0)
        if n:
            del node.path[max(0, len(node.path) - n):]      # (all of it if pruning ate into the guess)
        node.n_restored = 0
        self.replan_map()
        if node.get_parameter('restore_mode').value == 'path':
            self.restore_path()                      # legacy: extend the path (map mode: perception restores)

    def replan_map(self):
        node = self.node
        self.prune()
        if node.t_point is not None:
            return                          # final approach: the path to the T is fixed
        lock = []
        acc = 0.0
        for i, p in enumerate(node.path):
            if i:
                acc += math.hypot(p[0] - node.path[i - 1][0], p[1] - node.path[i - 1][1])
            lock.append(p)
            if acc >= 0.08:
                break
        if len(lock) >= 3 and acc >= 0.04:
            anchor = node.map.snap(*lock[-1], 0.03)
            relink = node.get_parameter('relink_radius').value
            if anchor is None and relink > 0.03:
                # the line seen now lies a few cm beside the remembered one (odometry drifted,
                # e.g. after a ramp): re-attach the path to what the map shows now
                anchor = node.map.snap(*lock[-1], relink)
            if anchor is None:
                return                      # map has nothing here (yet): keep the remembered path
            h = math.atan2(lock[-1][1] - lock[-3][1], lock[-1][0] - lock[-3][0])
        else:
            lock, anchor = self.acquire()
            if anchor is None:
                return
            h = node.pose[2]
        pts, end = node.map.trace(anchor[0], anchor[1], h)
        old_rest = max(0.0, path_len(node.path) - path_len(lock))
        if end == 'open' and path_len(pts) < old_rest - 0.10:
            return                          # the map lost part of the line: trust the memory
        new = lock[:-1] + pts if lock else pts
        if end == 'open' and path_len(pts) < 0.25:
            # the remembered line stops just ahead: if the camera sees a line right now
            # (the remembered one may be displaced by odometry error), follow what is seen
            link, anchor = self.acquire(vision.X_NEAR)
            if anchor is not None and min(math.hypot(anchor[0] - p[0], anchor[1] - p[1]) for p in new) < 0.10:
                pts2, end2 = node.map.trace(anchor[0], anchor[1], node.pose[2])
                if path_len(pts2) > path_len(pts) + 0.15 or end2 == 't':
                    new, pts, end = link + pts2, pts2, end2
        node.path = new
        node.path_end = end
        if end == 't':
            # the controller plugin may judge the finish itself (None = use the legacy rule)
            verdict = node.controller.confirm_finish(new, node.last_observation)
            if verdict is None:
                verdict = node.travel > node.get_parameter('finish_min_dist').value
            if verdict:
                # finish T seen: from now on drive this path to the T and stop there
                node.get_logger().info(f'finish T detected (driven {node.travel:.2f} m)')
                node.t_point = pts[-1]

    def restore_path(self):
        """The map has no more line ahead: continue along the learned route prediction."""
        node = self.node
        if node.restorer is None or node.t_point is not None or node.pose is None or node.travel < 0.3:
            return
        gp = node.get_parameter
        if len(node.path) >= 2 and (node.path_end != 'open' or path_len(node.path) >= gp('restore_trigger').value):
            return
        rp = [node.to_robot(*p) for p in node.path]
        if len(rp) >= 2:
            start, prev = rp[-1], rp[-2]
            heading = math.atan2(start[1] - prev[1], start[0] - prev[0])
        else:
            start, heading = (0.05, 0.0), 0.0
        prob = node.restorer.route(node.map, node.pose)
        ext = node.restorer.extend_ridge(prob, start, rp, gp('restore_len').value, gp('restore_thresh').value)
        dbg = os.environ.get('LF_RESTORE_DEBUG')
        if dbg and getattr(node, 'n_rdump', 0) < 12:
            node.n_rdump = getattr(node, 'n_rdump', 0) + 1
            line, seen = node.restorer.patch(node.map, node.pose)
            out_dir = Path(os.environ.get('ROS_LOG_DIR', '/tmp')).parent
            np.savez_compressed(out_dir / f'restore_{node.n_rdump:02d}.npz', line=line, seen=seen, prob=prob,
                                start=np.array(start), known=np.array(rp).reshape(-1, 2),
                                ext=np.array(ext).reshape(-1, 2), travel=node.travel)
        if dbg:
            node.get_logger().info(f'restore: path {path_len(node.path):.2f} m end={node.path_end} -> +{len(ext)} pts '
                                   f'pmax={float(prob.max()):.2f}')
        if len(ext) >= 3:
            node.path = list(node.path) + [node.to_odom(x, y) for x, y in ext]
            node.n_restored = len(ext)
            node.path_end = 'open'

