#!/usr/bin/env python3
"""Own a visible development session; stop only the recorded, identity-checked launch group."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

STATE = Path('/tmp/line_follower_gcs_session.json')


def identity(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError):
        return None


def stop(grace_sec=12.):
    try:
        data = json.loads(STATE.read_text())
    except (OSError, ValueError):
        print('No managed GCS session is running')
        return 0
    if identity(data['pid']) != data['identity']:
        print('No matching managed GCS launch remains')
        return 0
    os.kill(data['pid'], signal.SIGINT)
    deadline = time.monotonic()+grace_sec
    while time.monotonic()<deadline:
        if identity(data['pid']) != data['identity']:
            print('Managed GCS session stopped')
            return 0
        time.sleep(.1)
    if identity(data['pid']) == data['identity']:
        os.killpg(data['pid'], signal.SIGKILL)
    print('Managed GCS session escalated to owned-group SIGKILL')
    return 0


def main():
    if sys.argv[1:] == ['stop']:
        return stop()
    with STATE.open('a+') as state:
        try:
            fcntl.flock(state, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('A managed GCS session is already running', file=sys.stderr)
            return 1
        os.environ['ROS_DOMAIN_ID'] = '81'
        os.environ['GZ_PARTITION'] = f'gcs_visible_{os.getpid()}'
        child = subprocess.Popen(['ros2', 'launch', 'line_follower_bringup', 'dev_sim.launch.py',
                                  'headless:=true', 'gcs:=true', 'stop_on_gcs_exit:=true'] + sys.argv[1:], start_new_session=True)
        state.seek(0)
        state.truncate()
        json.dump({'pid': child.pid, 'identity': identity(child.pid)}, state)
        state.flush()
        def interrupted(*_):
            raise KeyboardInterrupt
        signal.signal(signal.SIGINT, interrupted)
        signal.signal(signal.SIGTERM, interrupted)
        try:
            return child.wait()
        except KeyboardInterrupt:
            return 0
        finally:
            try:
                os.kill(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pass
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            child.wait()
            state.seek(0)
            state.truncate()


if __name__ == '__main__':
    raise SystemExit(main())
