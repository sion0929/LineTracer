#!/usr/bin/env python3
"""DEV ONLY: run the line_follower node with the GROUND-TRUTH route instead of the camera map.

Never part of the submission (it reads Gazebo ground truth through gz-transport, which the
task forbids at runtime). It exists so the planning/control side can be developed and scored
on a perfect or realistically degraded route, independent of perception:

  ORACLE_LEVEL=0  perfect route: the true centre line ahead, exact relative geometry
  ORACLE_LEVEL=1  + limited view: only up to ORACLE_AHEAD (0.85 m) beyond the robot is known
  ORACLE_LEVEL=2  + perception-like errors: lateral noise (1.5 cm), 0.1 s latency, and an
                    along-track offset that jumps by 13-25 cm at sharp corners (as measured
                    for the camera map, experiments/results/lead/linemap)

The node's own perception, map and control run unchanged; only node.path / node.path_end are
replaced each camera frame. Used by bench.py like any follower:

  python3 experiments/bench.py MAPS --follower experiments/oracle/run_oracle.py --off 0.12 ...
  (set ORACLE_LEVEL / LF_PARAMS in the environment)
"""
import json
import math
import os
import sys
import threading
from collections import deque
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PKG = Path(os.environ.get("LF_PKG", HERE.parents[1] / "src" / "line_follower"))
sys.path.insert(0, str(PKG))
LEVEL = int(os.environ.get("ORACLE_LEVEL", "0"))
AHEAD = float(os.environ.get("ORACLE_AHEAD", "0.85"))
sys.argv = [sys.argv[0], "--ros-args", "--params-file",
            os.environ.get("LF_PARAMS", str(PKG / "config" / "follower.yaml")),
            "-p", "use_sim_time:=true", *sys.argv[1:]]

import rclpy  # noqa: E402
from line_follower import follower_node as fn  # noqa: E402


def load_route():
    """Centre line (world, ordered) of the map this bench job runs, via the job folder."""
    job_dir = Path(os.environ.get("ROS_LOG_DIR", ".")).parent
    job = json.loads((job_dir / "job.json").read_text())
    sdf = Path(job["map"])
    side = sdf.with_suffix(".json")
    if side.is_file():
        c = np.asarray(json.loads(side.read_text())["centerline"], float)[:, :2]
    elif (job_dir / "gt.npy").is_file():                    # official track (bench saves it)
        c = np.load(job_dir / "gt.npy")[:, :2]
    else:
        c = np.load(Path(__file__).resolve().parents[1] / "gt.npy")[:, :2]
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(c, axis=0), axis=1))]
    return c, arc


class Truth:
    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.pose = None
        self.lock = threading.Lock()
        self.node = Node()
        self.node.subscribe(Pose_V, "/world/line_track/dynamic_pose/info", self.cb)

    def cb(self, msg):
        for p in msg.pose:
            if p.name == "turtlebot3_waffle_pi":
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                with self.lock:
                    self.pose = (p.position.x, p.position.y, yaw)
                return


Base = fn.FollowerNode


class OracleNode(Base):
    def __init__(self):
        super().__init__()
        self.route, self.route_arc = load_route()
        self.truth = Truth()
        self.progress_s = 0.0
        self.rng = np.random.default_rng(int(os.environ.get("ORACLE_SEED", "0")))
        self.along_offset = 0.0
        self.last_heading = None
        self.history = deque(maxlen=5)                     # for latency at level 2
        self.get_logger().info(f"ORACLE route level {LEVEL} (dev only, uses ground truth)")

    def replan(self):
        with self.truth.lock:
            tp = self.truth.pose
        if tp is None or self.pose is None:
            return
        c, arc = self.route, self.route_arc
        # progress along the route (search near the previous value)
        lo = max(0, np.searchsorted(arc, self.progress_s - 0.4) - 1)
        hi = min(len(c), np.searchsorted(arc, self.progress_s + 0.6) + 1)
        d = np.linalg.norm(c[lo:hi] - tp[:2], axis=1)
        k = lo + int(np.argmin(d))
        self.progress_s = max(self.progress_s, float(arc[k]))
        end_s = arc[-1] if LEVEL == 0 else min(arc[-1], arc[k] + AHEAD + 0.25)
        sel = (arc >= arc[k] - 0.10) & (arc <= end_s)
        pts = c[sel]
        if LEVEL >= 2:
            # along-track jump at sharp heading changes (camera-map reference drift)
            h = math.atan2(*(c[min(k + 3, len(c) - 1)] - c[max(k - 3, 0)])[::-1])
            if self.last_heading is not None and abs(math.remainder(h - self.last_heading, math.tau)) > math.radians(60):
                self.along_offset += self.rng.uniform(0.13, 0.25)
            self.last_heading = h
            seg = np.diff(pts, axis=0, prepend=pts[:1])
            t = seg / np.maximum(np.linalg.norm(seg, axis=1, keepdims=True), 1e-9)
            t[0] = t[1] if len(t) > 1 else t[0]
            nrm = np.column_stack((-t[:, 1], t[:, 0]))
            pts = pts + nrm * self.rng.normal(0, 0.015, (len(pts), 1))   # lateral noise
            # the along-track drift is applied below as a shift of the route in the robot frame
        # world -> robot frame (truth), then robot -> odom with the node's own pose estimate
        cx, cy, cyaw = tp
        cs, sn = math.cos(cyaw), math.sin(cyaw)
        rel = np.column_stack(((pts[:, 0] - cx) * cs + (pts[:, 1] - cy) * sn,
                               -(pts[:, 0] - cx) * sn + (pts[:, 1] - cy) * cs))
        if LEVEL >= 1:
            rel = rel[rel[:, 0] > -0.10]
        if LEVEL >= 2 and len(rel):
            rel[:, 0] -= min(self.along_offset, 0.30) * 0.5   # route lags by part of the drift
        px, py, pyaw = self.pose
        c2, s2 = math.cos(pyaw), math.sin(pyaw)
        odom = [(px + x * c2 - y * s2, py + x * s2 + y * c2) for x, y in rel]
        if LEVEL >= 2:
            self.history.append(odom)
            odom = self.history[0]                       # ~0.1-0.15 s latency at 30-40 Hz
        if len(odom) < 2:
            return
        self.path = odom
        near_end = end_s >= arc[-1] - 1e-6
        self.path_end = "t" if near_end else "len"
        if near_end and self.t_point is None and arc[-1] - arc[k] < AHEAD:
            self.t_point = odom[-1]


fn.FollowerNode = OracleNode
_init = rclpy.init
rclpy.init = lambda *a, **k: _init(args=sys.argv)
fn.main()
