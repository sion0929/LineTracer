"""DEV ONLY ground-truth helpers shared by the oracle tools (never part of the submission).

load_route()  centre line of the map the current bench job runs (x, y, z) + arc length
Truth         robot pose from Gazebo through gz-transport (forbidden at runtime: dev only)
Progress      tracks the robot's arc position on the centre line
"""
import json
import math
import os
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


def _truth_proc(model, shared):
    """Child process: gz-transport subscriber (kept out of the rclpy process, where its
    callback thread occasionally crashed the node with SIGSEGV)."""
    import time
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V

    def cb(msg):
        for p in msg.pose:
            if p.name == model:
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
                stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
                with shared.get_lock():
                    shared[0], shared[1], shared[2], shared[3] = p.position.x, p.position.y, yaw, 1.0
                    shared[4], shared[5], shared[6] = pitch, p.position.z, stamp
                return

    node = Node()
    node.subscribe(Pose_V, "/world/line_track/dynamic_pose/info", cb)
    parent = os.getppid()
    while os.getppid() == parent:
        time.sleep(0.5)


class Truth:
    def __init__(self, model="turtlebot3_waffle_pi"):
        import multiprocessing as mp
        ctx = mp.get_context("spawn")
        self.shared = ctx.Array("d", 7)
        self.proc = ctx.Process(target=_truth_proc, args=(model, self.shared), daemon=True)
        self.proc.start()

    def get(self):
        with self.shared.get_lock():
            x, y, yaw, ok = self.shared[:4]
        return (x, y, yaw) if ok else None

    def get_full(self):
        """(x, y, yaw, pitch, z, sim stamp) or None."""
        with self.shared.get_lock():
            v = self.shared[:]
        return (v[0], v[1], v[2], v[4], v[5], v[6]) if v[3] else None


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
        # distance to the polyline, not to its vertices (the official gt.npy is 0.22 m sparse)
        seg = self.c[max(lo, k - 2):min(hi, k + 3)]
        off = float(seg_distance(np.asarray(xy[:2], float)[None], seg)[0]) if len(seg) >= 2 else float(d.min())
        return k, off


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
