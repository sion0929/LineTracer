#!/usr/bin/env python3
"""DEV ONLY: run the normal camera pipeline and log how wrong its route is, against ground truth.

The node drives exactly as in the submission (camera map, trace, controller). After every
replan the route it is about to follow is compared with the true centre line, both seen
from the robot (node path through the node's own pose estimate; centre line through the
Gazebo pose). This is the error that matters for following: drift that moves map and pose
together does not count, inconsistency between them does.

Per camera frame it records the cross-track distance of the route to the true line at
D = 0.15, 0.30, 0.45, 0.60, 0.80 m along the route (NaN where the route is shorter), plus
context (arc position, recent GT heading change, slope, path end, robot offset). Written to
<job>/shadow.npz; summarise with shadow_summary.py.

  LF_PARAMS=.../config/rule.yaml python3 experiments/bench.py MAPS \
      --follower experiments/oracle/shadow_route.py --off 0.12 --jobs 2 --output OUT

Use LF_PKG to point at another worktree's package (e.g. the DL branch). SHADOW_ORACLE=1 with
ORACLE_LEVEL=n logs the oracle route instead, so the oracle levels can be calibrated against
the real camera route on the same metric.
"""
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import gt  # noqa: E402
import run_oracle  # noqa: E402  (sets up the package path; its main() starts the node)

# SHADOW_ORACLE=1 measures the oracle route (ORACLE_LEVEL) the same way, for calibration
Base = run_oracle.OracleNode if os.environ.get("SHADOW_ORACLE") == "1" else run_oracle.fn.FollowerNode

D = np.array([0.15, 0.30, 0.45, 0.60, 0.80])
FIELDS = ("t", "travel", "s", "turn_back", "turn_ahead", "slope", "end", "plen", "robot_off", "pitch", "rock")


def resample(rp, dists):
    """Points at arc distances dists along the robot-frame route rp (starting from the robot)."""
    rp = np.asarray(rp, float)
    seg = np.linalg.norm(np.diff(rp, axis=0), axis=1)
    arc = np.r_[np.hypot(*rp[0]), np.hypot(*rp[0]) + np.cumsum(seg)]
    out = np.full((len(dists), 2), np.nan)
    for i, d in enumerate(dists):
        if d > arc[-1] or d < arc[0]:
            continue
        j = min(int(np.searchsorted(arc, d)), len(rp) - 1)
        if j == 0:
            out[i] = rp[0]
            continue
        f = (d - arc[j - 1]) / max(arc[j] - arc[j - 1], 1e-9)
        out[i] = rp[j - 1] + f * (rp[j] - rp[j - 1])
    return out, float(arc[-1])


class ShadowNode(Base):
    def __init__(self):
        super().__init__()
        self.route, self.route_arc = gt.load_route()
        head = np.unwrap(np.arctan2(*np.diff(self.route[:, :2], axis=0).T[::-1]))
        self.head = np.r_[head, head[-1]]
        dz = np.gradient(self.route[:, 2], self.route_arc, edge_order=1)
        self.slope = np.degrees(np.arctan(dz))
        self.truth = getattr(self, 'truth', None) or gt.Truth()
        self.prog = gt.Progress(self.route, self.route_arc)
        self.rows, self.errs = [], []
        self.out = gt.job_dir() / "shadow.npz"
        self.get_logger().info("SHADOW route logger (dev only, compares with ground truth)")

    def heading_change(self, k, a, b):
        """|heading change| of the true line over arc [s+a, s+b] around index k."""
        s = self.route_arc[k]
        i = int(np.searchsorted(self.route_arc, s + a))
        j = min(int(np.searchsorted(self.route_arc, s + b)), len(self.head) - 1)
        i = max(0, min(i, j))
        seg = self.head[i:j + 1]
        return float(np.degrees(seg.max() - seg.min())) if len(seg) else 0.0

    def replan(self):
        super().replan()
        tp = self.truth.get()
        if tp is None or self.pose is None:
            return
        k, robot_off = self.prog.update(tp[:2])
        s = self.route_arc[k]
        err = np.full(len(D), np.nan)
        plen = 0.0
        if len(self.path) >= 2:
            rp = [self.to_robot(*p) for p in self.path]
            pts, plen = resample(rp, D)
            win = (self.route_arc >= s - 0.3) & (self.route_arc <= s + 1.4)
            true_rel = gt.to_frame(self.route[win, :2], tp)
            ok = ~np.isnan(pts[:, 0])
            if ok.any() and len(true_rel) >= 2:
                err[ok] = gt.seg_distance(pts[ok], true_rel)
        t = self.get_clock().now().nanoseconds * 1e-9
        end = {"open": 0, "t": 1, "len": 2}.get(self.path_end, 3) if len(self.path) >= 2 else -1
        self.rows.append((t, self.travel, s, self.heading_change(k, -0.6, 0.0),
                          self.heading_change(k, 0.0, 0.8), float(self.slope[k]), end, plen,
                          robot_off, float(self.pitch_f), float(getattr(self, 'rock', np.nan))))
        self.errs.append(err)
        self.snapshot(plen, robot_off)
        if len(self.rows) % 60 == 0:
            self.save()

    def snapshot(self, plen, robot_off):
        """SHADOW_SNAP=1: dump BEV masks + map crop when the route runs short before a turn
        (the 'route stops at a sharp corner' failure), at most 8 per run, 0.3 s apart (SHADOW_SNAP_PLEN sets the length threshold)."""
        if os.environ.get("SHADOW_SNAP") != "1" or getattr(self, "n_snap", 0) >= 8:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        if plen > float(os.environ.get("SHADOW_SNAP_PLEN", "0.20")) or self.rows[-1][4] < 60 \
                or t - getattr(self, "t_snap", -9) < 0.3:
            return
        from line_follower.line_map import C0, RES
        self.n_snap = getattr(self, "n_snap", 0) + 1
        self.t_snap = t
        px, py, _ = self.pose
        ix, iy, r = round(px / RES) + C0, round(py / RES) + C0, round(1.0 / RES)
        obs = self.last_observation
        np.savez_compressed(gt.job_dir() / f"snap{self.n_snap}.npz",
                            grid=np.asarray(self.map.grid[iy - r:iy + r + 1, ix - r:ix + r + 1]),
                            origin=np.array([(ix - r - C0) * RES, (iy - r - C0) * RES]), res=RES,
                            pose=np.array(self.pose), truth=np.array(self.truth.get() or (0, 0, 0)),
                            path=np.asarray(self.path, float).reshape(-1, 2),
                            line=np.asarray(obs.line) if obs is not None else np.zeros(1),
                            valid=np.asarray(obs.valid) if obs is not None else np.zeros(1),
                            route=self.route[:, :2], t=t, plen=plen, robot_off=robot_off)

    def save(self):
        if not self.rows:
            return
        rows = np.asarray(self.rows, float)
        tmp = self.out.with_suffix(".tmp.npz")
        np.savez(tmp, err=np.asarray(self.errs), D=D, fields=np.array(FIELDS),
                 **{f: rows[:, i] for i, f in enumerate(FIELDS)})
        os.replace(tmp, self.out)

    def destroy_node(self):
        try:
            self.save()
        finally:
            return super().destroy_node()


if __name__ == "__main__":
    run_oracle.main(ShadowNode)
