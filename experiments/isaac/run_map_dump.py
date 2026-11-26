#!/usr/bin/env python3
"""Run the follower with Isaac truth for map placement and export map_world.npz."""

import json
import math
import os
from pathlib import Path
import signal
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parents[1] / "src/line_follower"


def world_xy(xy, origin, *, inverse=False):
    """Transform XY arrays between the start-relative odom frame and world."""
    x, y, yaw = origin
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    return (xy - [x, y]) @ rotation if inverse else xy @ rotation.T + [x, y]


def stamp_ns(msg):
    stamp = msg.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main():
    sys.path[:0] = [str(PKG), str(HERE.parent / "user_base")]
    import rclpy
    from line_follower import follower_node as fn, vision
    from line_follower.line_map import C0, RES
    from ros_args import node_scoped

    output = Path(os.environ["ISAAC_DUMP_DIR"])
    job = json.loads((output / "job.json").read_text())
    origin = job["odom_origin"]
    spawn_z = job["spawn_z"]
    overlay = output / "overlay.yaml"
    ros_args = ["--ros-args", "--params-file", os.environ["LF_PARAMS"]]
    if overlay.is_file():
        ros_args += ["--params-file", str(overlay)]
    ros_args += node_scoped([
        *sys.argv[1:], "-p", "use_sim_time:=true", "-p", "scan_match:=false",
        "-r", "follower_node:/odom:=/isaac/true_odom",
    ])

    class Dumping(fn.FollowerNode):
        def __init__(self):
            super().__init__()
            self._poses = {}
            self._images = {}
            self._last_pair = -1
            self._cells = {}
            self._restored = {}
            self._track = []
            self.create_timer(2.0, self._save)

        def on_odom(self, msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = 2 * math.atan2(q.z, q.w)
            x, y = world_xy(np.array([p.x, p.y]), origin, inverse=True)
            heading = math.remainder(yaw - origin[2], math.tau)
            if self.pose is not None:
                self.travel += math.hypot(x - self.pose[0], y - self.pose[1])
            self.pose = (x, y, heading)
            self.elev = p.z - spawn_z
            self.odom_xy, self.odom_yaw = (x, y), heading
            ns = stamp_ns(msg)
            now = ns * 1e-9
            self.hist.append((now, self.pose))
            if len(self.hist) > 100:
                del self.hist[:50]
            self._track.append((now, p.x, p.y, yaw, x, y, heading, self.travel))
            self._poses[ns] = (self.pose, self.elev, self.travel)
            self._process_pair(ns)

        def on_image(self, msg):
            ns = stamp_ns(msg)
            if ns <= self._last_pair:
                return
            self._images[ns] = msg
            self._process_pair(ns)

        def _process_pair(self, ns):
            # The two DDS topics can be dispatched in either order.
            for pending in (self._images, self._poses):
                if len(pending) > 8:
                    del pending[min(pending)]
            if ns not in self._images or ns not in self._poses:
                return
            msg = self._images.pop(ns)
            pose, elev, travel = self._poses.pop(ns)
            if ns <= self._last_pair:
                return
            latest = self.pose, self.elev, self.travel, self.done
            self.pose, self.elev, self.travel, self.done = pose, elev, travel, False
            self.map.last = None
            restore_stamp = getattr(self.perception_stage, "t_restore", None)
            try:
                super().on_image(msg)
                self._collect(ns * 1e-9, restore_stamp)
                self._last_pair = ns
            finally:
                self.pose, self.elev, self.travel, self.done = latest

        def pose_at(self, _ts):
            # While processing a matched image, self.pose is its exact capture pose.
            return self.pose

        def _collect(self, now, restore_stamp):
            if self.map.last is not None:
                ox, oy, _, seen = self.map.last
                height, width = seen.shape
                grid = self.map.grid[oy:oy + height, ox:ox + width]
                rows, cols = np.nonzero(seen & (grid > 60))
                rows, cols = rows + oy, cols + ox
                xy = np.column_stack((cols - C0, rows - C0)) * RES
                keys = np.rint(world_xy(xy, origin) / RES).astype(int)
                dist = np.linalg.norm(xy - self.pose[:2], axis=1)
                conf = self.map.conf[rows, cols] / 255.0
                for key, d, c in zip(map(tuple, keys), dist, conf):
                    self._cells.setdefault(key, (now, d, self.travel, c))
            if (self.map.restored is not None and
                    getattr(self.perception_stage, "t_restore", None) != restore_stamp):
                rows, cols = np.nonzero(self.map.restored)
                xy = np.column_stack((cols - C0, rows - C0)) * RES
                keys = np.rint(world_xy(xy, origin) / RES).astype(int)
                for key in map(tuple, keys):
                    self._restored.setdefault(key, (now, self.travel))

        def _save(self):
            xy = np.asarray(list(self._cells), float).reshape(-1, 2) * RES
            vals = np.asarray(list(self._cells.values()), float).reshape(-1, 4)
            pending = output / "map_world.tmp.npz"
            np.savez_compressed(
                pending, xy=xy, t=vals[:, 0], dist=vals[:, 1], travel=vals[:, 2],
                conf=vals[:, 3], track=np.asarray(self._track, float).reshape(-1, 8),
                x_far=vision.X_FAR, far_stats=np.asarray(getattr(self, "far_stats", [0, 0])),
                rxy=np.asarray(list(self._restored), float).reshape(-1, 2) * RES,
                rinfo=np.asarray(list(self._restored.values()), float).reshape(-1, 2),
            )
            pending.replace(output / "map_world.npz")

        def destroy_node(self):
            try:
                if self._last_pair < 0:
                    raise RuntimeError("No timestamp-matched Isaac camera and truth frames")
                self._save()
            finally:
                super().destroy_node()

    def stop(_signal, _frame):
        raise KeyboardInterrupt

    init = rclpy.init

    def init_with_args():
        from rclpy.signals import SignalHandlerOptions

        init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
        signal.signal(signal.SIGTERM, stop)

    fn.FollowerNode = Dumping
    rclpy.init = init_with_args
    fn.main()


if __name__ == "__main__":
    main()
