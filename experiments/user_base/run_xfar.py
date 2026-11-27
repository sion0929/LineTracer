#!/usr/bin/env python3
"""DEV ONLY bench follower: the node with BEV range X_FAR = --xfar (env LF_X_FAR), plus a map logger.

Every 0.5 s the map cells near the robot (within X_FAR + 0.4 m) are moved to the world frame with
the CURRENT truth-vs-odom transform (so global odometry drift does not count, only what the
camera recorded), and accumulated: world cell -> (first time, robot distance when first in the
map, truth progress). Written to <job>/map_world.npz. Ground truth is used for logging only.
"""
import os, sys, threading, time
from pathlib import Path
args = sys.argv[1:]
VIDEO = False
while args[:1] in (['--xfar'], ['--video']):
    if args[0] == '--xfar':
        os.environ['LF_X_FAR'] = args[1]
        args = args[2:]
    else:
        VIDEO = True
        args = args[1:]
HERE = Path(__file__).resolve().parent
PKG = HERE.parents[1] / 'src/line_follower'
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(HERE))
from ros_args import node_scoped  # noqa: E402
sys.path.insert(0, str(HERE.parent / 'oracle'))
params = os.environ.get('LF_PARAMS', str(PKG / 'config/rule.yaml'))
sys.argv = [sys.argv[0], '--ros-args', '--params-file', params, '-p', 'use_sim_time:=true', *node_scoped(args)]
_parent = os.getppid()


def _watch():
    while os.getppid() == _parent:
        time.sleep(0.5)
    os._exit(0)


if __name__ == '__main__':
    threading.Thread(target=_watch, daemon=True).start()
import rclpy  # noqa: E402
from line_follower import follower_node as fn  # noqa: E402
from map_logger import MapLogger  # noqa: E402


class Logger(MapLogger, fn.FollowerNode):
    log_video = VIDEO


if __name__ == '__main__':          # the truth child process (spawn) re-imports this file
    fn.FollowerNode = Logger
    _init = rclpy.init
    rclpy.init = lambda *a, **k: _init(args=sys.argv)
    fn.main()
