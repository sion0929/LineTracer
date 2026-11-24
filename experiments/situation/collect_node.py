#!/usr/bin/env python3
"""DEV ONLY: the package follower node, unchanged, plus a situation-data logger (bench follower).

Drives exactly like the submission (LF_PARAMS, camera line map, trace, controller) and records,
at 5 Hz and at every replan whose path ends in a T bar, what the situation features need in the
robot frame (map patch, traced path, path end, pitch, view limit, start position) together with
the node's own pose and the Gazebo ground-truth pose. Labels are computed offline
(collect.py build) from the map sidecar JSON, so label definitions can change without re-driving.

The finish verdict is taken from the ground truth here (dev only): a T candidate is accepted when
it lies within 0.15 m of the real route end and
is less than 0.40 m ahead of the robot. This keeps the run going past false T bars so the
whole segment is recorded, and still stops at the finish like a perfect judge would.

  LF_PARAMS=.../config/rule.yaml python3 experiments/bench.py MAPS \
      --follower experiments/situation/collect_node.py --off 0.2 --output OUT
Writes <job>/situation.npz.
"""
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "oracle"))
import gt  # noqa: E402
import run_oracle  # noqa: E402  (puts the package on sys.path; main() starts the node)

from line_follower.situation import features as FE  # noqa: E402

NPTS = 64


class CollectNode(run_oracle.fn.FollowerNode):
    def __init__(self):
        super().__init__()
        self.truth = gt.Truth()
        job = json.loads((gt.job_dir() / "job.json").read_text())
        meta = json.loads(Path(job["map"]).with_suffix(".json").read_text())
        self.finish_world = np.asarray(meta["centerline"][-1][:2], float)
        self.out = gt.job_dir() / "situation.npz"
        self.rows = []
        self.start = None
        self.create_timer(0.2, lambda: self.record(0))
        ctl = self.controller
        ctl.confirm_finish = self.gt_finish
        self.get_logger().info("SITUATION logger (dev only, ground-truth finish verdict)")

    def gt_finish(self, path, observation):
        tp = self.truth.get()
        if tp is None:
            return False
        rx, ry = self.to_robot(*path[-1])
        wx = tp[0] + rx * math.cos(tp[2]) - ry * math.sin(tp[2])
        wy = tp[1] + rx * math.sin(tp[2]) + ry * math.cos(tp[2])
        # accept only once the bar is close, so the approach to the real T is recorded too
        return bool(np.hypot(wx - self.finish_world[0], wy - self.finish_world[1]) < 0.15
                    and math.hypot(rx, ry) < 0.40)

    def replan(self):
        fixed = self.t_point is not None
        super().replan()
        if not fixed and self.path_end == "t" and len(self.path) >= 2:
            self.record(1)

    def record(self, tcand):
        if self.pose is None:
            return
        if self.start is None:
            self.start = self.pose[:2]
        tp = self.truth.get()
        if tp is None:
            return
        rp = np.full((NPTS, 2), np.nan, np.float32)
        n = 0
        if len(self.path) >= 2:
            pts = FE._to_robot(self.path[:NPTS], self.pose)
            n = len(pts)
            rp[:n] = pts
        st = FE._to_robot([self.start], self.pose)[0]
        patch = FE.robot_patch(self.map.grid, self.pose)
        end = {"open": 0, "t": 1, "len": 2}.get(self.path_end, 3) if n >= 2 else -1
        self.rows.append(dict(
            t=self.get_clock().now().nanoseconds * 1e-9, tcand=tcand, patch=np.packbits(patch.ravel()),
            rp=rp, n=n, end=end, pitch=self.pitch_f, x_far=FE.node_x_far(self), start=st,
            pose=np.asarray(self.pose, float), gt=np.asarray(tp, float), travel=self.travel,
            v=self.v_cmd, w=self.w_cmd, done=int(self.done)))
        if len(self.rows) % 50 == 0:
            self.save()

    def save(self):
        if not self.rows:
            return
        keys = self.rows[0].keys()
        tmp = self.out.with_suffix(".tmp.npz")
        np.savez_compressed(tmp, **{k: np.stack([np.asarray(r[k]) for r in self.rows]) for k in keys})
        os.replace(tmp, self.out)

    def destroy_node(self):
        try:
            self.save()
        finally:
            return super().destroy_node()


if __name__ == "__main__":
    run_oracle.main(CollectNode)
