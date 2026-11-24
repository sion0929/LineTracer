#!/usr/bin/env python3
"""Bench wrapper for the user's line_follower package (unchanged code + follower.yaml)."""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..', '..', 'src', 'line_follower')
sys.path.insert(0, PKG)
extra = sys.argv[1:]
sys.argv = [sys.argv[0], '--ros-args', '--params-file', os.environ.get('LF_PARAMS', os.path.join(PKG, 'config', 'follower.yaml')),
            '-p', 'use_sim_time:=true', *extra]
import threading, time
_parent = os.getppid()


def _watch():
    # a follower outliving its bench would keep driving the next run's robot on this domain
    while os.getppid() == _parent:
        time.sleep(0.5)
    os._exit(0)


threading.Thread(target=_watch, daemon=True).start()
import rclpy
_init = rclpy.init
rclpy.init = lambda *a, **k: _init(args=sys.argv)
from line_follower.follower_node import main
main()
