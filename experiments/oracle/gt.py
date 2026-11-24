"""DEV ONLY ground-truth helpers shared by the oracle tools (never part of the submission).

load_route()  centre line of the map the current bench job runs (x, y, z) + arc length
Truth         robot pose from Gazebo through gz-transport (forbidden at runtime: dev only)
Progress      tracks the robot's arc position on the centre line
"""
import json
import math
import os
import threading
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def job_dir():
    return Path(os.environ.get("ROS_LOG_DIR", ".")).parent


def load_route():
    """Centre line (world, ordered, columns x y z) of this bench job's map and its arc length."""
    folder = job_dir()
    job = json.loads((folder / "job.json").read_text())
    side = Path(job["map"]).with_suffix(".json")
    if side.is_file():
        c = np.asarray(json.loads(side.read_text())["centerline"], float)
    elif (folder / "gt.npy").is_file():                    # official track (bench saves it)
        c = np.load(folder / "gt.npy")[:, :2]
    else:
        c = np.load(HERE.parent / "gt.npy")[:, :2]
    if c.shape[1] == 2:
        c = np.column_stack((c, np.zeros(len(c))))
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(c[:, :2], axis=0), axis=1))]
    return c, arc


class Truth:
    def __init__(self, model="turtlebot3_waffle_pi"):
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.model = model
        self.pose = None
        self.lock = threading.Lock()
        self.node = Node()
        self.node.subscribe(Pose_V, "/world/line_track/dynamic_pose/info", self.cb)

    def cb(self, msg):
        for p in msg.pose:
            if p.name == self.model:
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                with self.lock:
                    self.pose = (p.position.x, p.position.y, yaw)
                return

    def get(self):
        with self.lock:
            return self.pose


class Progress:
    """Monotone arc position of the robot on the centre line (searched near the last value)."""

    def __init__(self, route, arc):
        self.c, self.arc, self.s = route[:, :2], arc, 0.0

    def update(self, xy):
        lo = max(0, np.searchsorted(self.arc, self.s - 0.4) - 1)
        hi = min(len(self.c), np.searchsorted(self.arc, self.s + 0.6) + 1)
        d = np.linalg.norm(self.c[lo:hi] - np.asarray(xy[:2]), axis=1)
        k = lo + int(np.argmin(d))
        self.s = max(self.s, float(self.arc[k]))
        return k, float(d.min())


def to_frame(pts, pose):
    """World points (N, 2) -> frame of pose (x, y, yaw)."""
    cx, cy, cyaw = pose
    cs, sn = math.cos(cyaw), math.sin(cyaw)
    dx, dy = pts[:, 0] - cx, pts[:, 1] - cy
    return np.column_stack((dx * cs + dy * sn, -dx * sn + dy * cs))


def seg_distance(p, poly):
    """Distance of points p (M, 2) to the polyline poly (N, 2)."""
    a, b = poly[:-1], poly[1:]
    ab = b - a
    t = np.einsum("mnk,nk->mn", p[:, None, :] - a[None], ab) / np.maximum(np.einsum("nk,nk->n", ab, ab), 1e-12)
    t = np.clip(t, 0.0, 1.0)
    q = a[None] + t[..., None] * ab[None]
    return np.linalg.norm(p[:, None, :] - q, axis=2).min(axis=1)
