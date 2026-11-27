#!/usr/bin/env python3
"""Bench wrapper, split mode: the user's line_follower as four nodes (docs/architecture/NODE_SPLIT.md).

Counterpart of run_user.py with the same contract: LF_PARAMS (default follower.yaml), LF_X_FAR,
LF_PERCEPTION_LOG, extra ROS args (`-p name:=value`, `--params-file X`) on the command line.
Each params file is rewritten so its follower_node section reaches every split node, and each
`-p` is scoped to the node (ros_args.for_node, R7). perception_node, map_node, planning_node and
control_node run as separate processes; when one exits or this wrapper is stopped, all stop.
"""
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..', '..', 'src', 'line_follower')
sys.path.insert(0, PKG)
sys.path.insert(0, HERE)
from ros_args import for_node  # noqa: E402

MODULES = {
    'perception_node': 'line_follower.split.perception_node',
    'map_node': 'line_follower.split.map_node',
    'planning_node': 'line_follower.split.planning_node',
    'control_node': 'line_follower.split.control_node',
}


def run_child(name, ros_args):
    parent = os.getppid()

    def watch():
        # a node outliving its bench would keep driving the next run's robot on this domain
        while os.getppid() == parent:
            time.sleep(0.5)
        os._exit(0)
    threading.Thread(target=watch, daemon=True).start()
    sys.argv = [sys.argv[0], *ros_args]
    import importlib
    import rclpy
    init = rclpy.init
    rclpy.init = lambda *a, **k: init(args=sys.argv)
    importlib.import_module(MODULES[name]).main()


def with_all_node_params(args, out_dir):
    from line_follower.split.params import write_all_nodes
    out, i = [], 0
    while i < len(args):
        if args[i] == '--params-file' and i + 1 < len(args):
            out += ['--params-file', write_all_nodes(args[i + 1], out_dir)]
            i += 2
        else:
            out.append(args[i])
            i += 1
    return out


def main():
    if len(sys.argv) > 2 and sys.argv[1] == '--child':
        return run_child(sys.argv[2], sys.argv[3:])
    tmp = tempfile.mkdtemp(prefix='lf_split_')
    params = os.environ.get('LF_PARAMS', os.path.join(PKG, 'config', 'follower.yaml'))
    base = with_all_node_params(['--params-file', params, '-p', 'use_sim_time:=true', *sys.argv[1:]], tmp)
    procs = {}
    stopping = []

    def stop(*_):
        stopping.append(True)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    status = 0
    try:
        for name in MODULES:
            args = for_node(base, name)
            if '--ros-args' not in args:
                args = ['--ros-args', *args]
            procs[name] = subprocess.Popen([sys.executable, os.path.abspath(__file__), '--child', name, *args])
        while not stopping:
            ended = [(n, p.returncode) for n, p in procs.items() if p.poll() is not None]
            if ended:
                name, status = ended[0]
                print(f'run_split: {name} exited with {status}, stopping the others', file=sys.stderr)
                break
            time.sleep(0.2)
    finally:
        for p in procs.values():
            if p.poll() is None:
                p.send_signal(signal.SIGINT)      # rclpy shuts down cleanly (perception log flush)
        deadline = time.time() + 5.0
        for p in procs.values():
            try:
                p.wait(timeout=max(0.1, deadline - time.time()))
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        shutil.rmtree(tmp, ignore_errors=True)
    return status


if __name__ == '__main__':
    sys.exit(main())
