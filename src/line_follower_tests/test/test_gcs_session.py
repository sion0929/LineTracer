"""Owned PID identity and forced cleanup, without ROS/Gazebo or other processes."""
import importlib.util
import json
from pathlib import Path
import signal
import subprocess
import sys

MODULE = Path(__file__).resolve().parents[3]/'tools/gcs_session.py'
spec = importlib.util.spec_from_file_location('gcs_session', MODULE)
session = importlib.util.module_from_spec(spec)
spec.loader.exec_module(session)


def test_stop_escalates_only_owned_group(tmp_path):
    child = subprocess.Popen([sys.executable, '-u', '-c',
        'import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); print("ready",flush=True); time.sleep(20)'],
        stdout=subprocess.PIPE, text=True, start_new_session=True)
    try:
        assert child.stdout.readline().strip()=='ready'
        session.STATE = tmp_path/'session.json'
        session.STATE.write_text(json.dumps({'pid':child.pid,'identity':session.identity(child.pid)}))
        assert session.stop(.1)==0
        assert child.wait(timeout=1)==-signal.SIGKILL
    finally:
        if child.poll() is None:
            child.kill()
        child.wait()


def test_wrong_pid_identity_is_not_signalled(tmp_path):
    child = subprocess.Popen([sys.executable,'-c','import time; time.sleep(20)'], start_new_session=True)
    try:
        session.STATE = tmp_path/'session.json'
        session.STATE.write_text(json.dumps({'pid':child.pid,'identity':'wrong-start-time'}))
        assert session.stop(.1)==0
        assert child.poll() is None
    finally:
        child.kill()
        child.wait()
