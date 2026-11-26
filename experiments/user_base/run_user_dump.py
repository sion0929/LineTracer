#!/usr/bin/env python3
"""Run the user's line_follower node unchanged, but dump its line map every 2 s (dev tool).

Saves LF_DUMP (npz): grid cells that hold line (odom frame, metres), robot odom pose
history as estimated by the node, travel. Nothing here feeds back into the robot.
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.environ.get('LF_PKG', os.path.join(HERE, '..', '..', 'src', 'line_follower'))
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ros_args import node_scoped  # noqa: E402
DUMP = os.environ.get('LF_DUMP', 'line_map_dump.npz')
sys.argv = [sys.argv[0], '--ros-args', '--params-file',
            os.environ.get('LF_PARAMS', os.path.join(PKG, 'config', 'follower.yaml')),
            '-p', 'use_sim_time:=true', *node_scoped(sys.argv[1:])]
import numpy as np
import rclpy
from line_follower import follower_node as fn
from line_follower.line_map import RES, C0

Base = fn.FollowerNode


class Dumping(Base):
    def __init__(self):
        super().__init__()
        self._poses = []
        self.create_timer(0.1, self._log_pose)
        self.create_timer(2.0, self._dump)

    def _log_pose(self):
        if self.pose is not None:
            self._poses.append((self.get_clock().now().nanoseconds * 1e-9, *self.pose, self.travel))

    def _dump(self):
        ii, jj = np.nonzero(self.map.grid)
        xy = np.column_stack(((jj - C0) * RES, (ii - C0) * RES)).astype(np.float32)
        tmp = DUMP + '.tmp.npz'
        np.savez_compressed(tmp, cells=xy, poses=np.array(self._poses, np.float64), res=RES)
        os.replace(tmp, DUMP)

    def destroy_node(self):
        try:
            self._dump()
        finally:
            super().destroy_node()


fn.FollowerNode = Dumping
_init = rclpy.init
rclpy.init = lambda *a, **k: _init(args=sys.argv)
fn.main()
