#!/usr/bin/env python3
"""Guarded stack for bench / safety scripts: follower + safety_guard, drop-in for run_user.py.

Children (same process group, so `killpg` from a harness reaches all three):
  follower  run_user.py (unchanged follower code + LF_PARAMS) with /cmd_vel remapped to the
            internal candidate topic /line_follower/cmd_raw
  guard     line_follower.safety_guard (source tree, like run_user.py): the only /cmd_vel
            publisher

  python3 experiments/bench.py <maps> --follower experiments/user_base/run_guarded.py ...
  LF_PARAMS=src/line_follower/config/rule.yaml python3 experiments/user_base/run_guarded.py [ros args]

Command-line arguments go to the follower, as with run_user.py. Environment:
  LF_PARAMS          follower parameter file (run_user.py default: config/follower.yaml)
  LF_SAFETY_PARAMS   guard parameter file (default config/safety_sim.yaml)
  LF_START_DELAY     guard start gate in wall seconds (default 0: like run_user.py, harnesses
                     start this after the simulator is up; 20 reproduces the launch file)
  LF_GUARD_ARGS      extra ROS arguments for the guard, e.g. '-r /odom:=/odom_relay'
  LF_GUARDED_PIDS    write {"parent", "follower", "guard"} process ids to this JSON file

SIGINT / SIGTERM are forwarded to both children; this wrapper exits (code 0) once both have
exited. If the follower dies on its own the guard keeps running and holds zero (that is the
point). If the guard dies on its own the follower is stopped too and the exit code is 1.
Children receive SIGTERM if this wrapper dies (PR_SET_PDEATHSIG), so even a SIGKILL of the
wrapper alone ends with the guard publishing zero.
"""
import ctypes
import json
import os
import shlex
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, '..', '..', 'src', 'line_follower'))
RUN_USER = os.path.join(HERE, 'run_user.py')
CANDIDATE = '/line_follower/cmd_raw'
KILL_AFTER = 8.0            # s after the stop request before children are SIGKILLed


def _die_with_parent():
    try:
        ctypes.CDLL('libc.so.6', use_errno=True).prctl(1, signal.SIGTERM)   # PR_SET_PDEATHSIG
    except (OSError, AttributeError):
        pass


def guard_main():
    """Child mode: run the guard from the source tree (ROS args are in sys.argv)."""
    sys.path.insert(0, PKG)
    from line_follower.safety_guard import main
    main()


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--guard':
        return guard_main()
    delay = float(os.environ.get('LF_START_DELAY') or 0.0)
    safety = os.environ.get('LF_SAFETY_PARAMS') or os.path.join(PKG, 'config', 'safety_sim.yaml')
    guard_cmd = [sys.executable, os.path.abspath(__file__), '--guard', '--ros-args',
                 '--params-file', safety, '-p', 'use_sim_time:=true', '-p', f'start_delay:={delay!r}',
                 *shlex.split(os.environ.get('LF_GUARD_ARGS', ''))]
    follower_cmd = [sys.executable, RUN_USER, '-r', f'/cmd_vel:={CANDIDATE}', *sys.argv[1:]]

    children = {}
    stop_at = [None]

    def forward(signum, _frame):
        if stop_at[0] is None:
            stop_at[0] = time.monotonic()
        for proc in children.values():
            if proc.poll() is None:
                try:
                    proc.send_signal(signum)
                except ProcessLookupError:
                    pass

    signal.signal(signal.SIGINT, forward)
    signal.signal(signal.SIGTERM, forward)
    children['guard'] = subprocess.Popen(guard_cmd, preexec_fn=_die_with_parent)
    children['follower'] = subprocess.Popen(follower_cmd, preexec_fn=_die_with_parent)
    pidfile = os.environ.get('LF_GUARDED_PIDS')
    if pidfile:
        tmp = pidfile + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(dict(parent=os.getpid(), **{k: p.pid for k, p in children.items()}), f)
        os.replace(tmp, pidfile)
    print(f'run_guarded: guard pid {children["guard"].pid}, follower pid {children["follower"].pid}, '
          f'start gate {delay:.1f} s, safety params {safety}', flush=True)

    code = 0
    reported = set()
    while True:
        for name, proc in children.items():
            if proc.poll() is not None and name not in reported:
                reported.add(name)
                print(f'run_guarded: {name} exited with code {proc.returncode}', flush=True)
        if len(reported) == len(children):
            break
        if 'guard' in reported and stop_at[0] is None:
            print('run_guarded: guard exited on its own: stopping the follower', flush=True)
            code = 1
            forward(signal.SIGINT, None)
        if stop_at[0] is not None and time.monotonic() - stop_at[0] > KILL_AFTER:
            for proc in children.values():
                if proc.poll() is None:
                    proc.kill()
        time.sleep(0.05)
    return code


if __name__ == '__main__':
    sys.exit(main())
