"""Development recorder: owns one rosbag process group and a per-run audit sidecar."""
import hashlib
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import threading
import time
import uuid
from datetime import datetime, timezone

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from line_follower_interfaces.msg import MissionState, RecordingStatus
from line_follower_interfaces.srv import BagCommand
from .bag_logic import RequestCache


def record_topics():
    share = Path(get_package_share_directory('line_follower_bringup'))
    return json.loads((share/'config/record_topics.json').read_text())


def fingerprint(root, params):
    digest = hashlib.sha256()
    files = sorted(p for p in (root/'src').rglob('*') if p.is_file() and
                   p.suffix in {'.cpp','.hpp','.h','.py','.yaml','.yml','.json','.xml','.msg','.srv','.txt'}
                   and not {'models','worlds','__pycache__'}.intersection(p.parts))
    for path in files:
        digest.update(str(path.relative_to(root)).encode())
        digest.update(path.read_bytes())
    digest.update(params.read_bytes())
    return digest.hexdigest()


class GcsRecorder(Node):
    def __init__(self):
        super().__init__('gcs_recorder')
        self.root = Path(self.declare_parameter('output_root', '/ws/bags').value)
        self.source = Path(self.declare_parameter('source_root', '/ws').value)
        self.max_sec = self.declare_parameter('max_record_sec', 300.).value
        self.max_bytes = self.declare_parameter('max_record_bytes', 1024*1024*1024).value
        self.auto = self.declare_parameter('record_on_start', False).value
        self.run_id = ''
        self.child = self.log = None
        self.bag = None
        self.state, self.reason = 'IDLE', 'not recording'
        self.metadata = {}
        self.cache = RequestCache()
        self.seen = {}
        self.status_pub = self.create_publisher(RecordingStatus, '/gcs/recording_status', 10)
        self.create_subscription(MissionState, '/mission/state', self.mission, 10)
        self.create_service(BagCommand, '/gcs/recorder/command', self.command)
        self.create_timer(.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def mission(self, msg):
        self.run_id = msg.run_id

    def stamp(self):
        return {'ros_ns': self.get_clock().now().nanoseconds,
                'utc': datetime.now(timezone.utc).isoformat(), 'wall_monotonic': time.monotonic()}

    def write_metadata(self):
        path = self.bag/'run_metadata.json'
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.metadata, indent=2, sort_keys=True)+'\n')
        temporary.replace(path)

    def start_recording(self):
        self.root.mkdir(parents=True, exist_ok=True)
        self.bag = self.root/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'_'+uuid.uuid4().hex[:8])
        # rosbag creates the directory itself. The sibling log is dev output, not part of the bag.
        self.log = self.bag.with_suffix('.recorder.log').open('w')
        params = Path(get_package_share_directory('line_follower_bringup'))/'config/competition.yaml'
        self.metadata = {'schema_version': 1, 'run_id': self.run_id,
            'use_sim_time': self.get_parameter('use_sim_time').value, 'start': self.stamp(),
            'recorder_requests': [], 'stop': None, 'closed_cleanly': False, 'topics': record_topics(),
            'code_config_sha256': fingerprint(self.source, params), 'params_file': str(params),
            'environment': {'ros_distro': os.getenv('ROS_DISTRO'), 'ros_domain_id': os.getenv('ROS_DOMAIN_ID'),
                'gz_partition': os.getenv('GZ_PARTITION'), 'platform': platform.platform(),
                'hostname': platform.node(), 'storage': 'mcap', 'compression': 'zstd_fast'}}
        self.started = time.monotonic()
        self.child = subprocess.Popen(['ros2','bag','record','--output',str(self.bag),
            '--storage','mcap','--storage-preset-profile','zstd_fast', '--disable-keyboard-controls',
            '--use-sim-time', '--max-cache-size','20000000', '--topics']+record_topics(),
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        self.state, self.reason = 'STARTING', 'recorder process started'

    def stop_recording(self, reason='stop requested'):
        if self.child is None:
            return False, 'not recording'
        child = self.child
        try:
            os.killpg(child.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        forced = False
        try:
            child.wait(timeout=6.)
        except subprocess.TimeoutExpired:
            forced = True
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()
        self.log.close()
        self.log = None
        self.child = None
        # Both a clean process exit and rosbag's metadata are required.
        clean = not forced and child.returncode == 0 and (self.bag/'metadata.yaml').exists()
        self.metadata.update(stop=self.stamp(), closed_cleanly=clean, stop_reason=reason,
                             recorder_exit_code=child.returncode)
        if self.bag.exists():
            self.write_metadata()
        self.state = 'STOPPED' if clean else 'FAILED'
        self.reason = reason if clean else f'unclean recorder stop: exit={child.returncode}, forced={forced}'
        return clean, self.reason

    def command(self, req, res):
        if req.request_id in self.seen and self.seen[req.request_id] != req.command:
            res.accepted, res.reason = False, 'request_id reused with different command'
            return res
        old = self.cache.get(req.request_id)
        if old is not None:
            res.accepted, res.reason = old
            return res
        if not req.request_id:
            answer = (False, 'request_id required')
        elif req.request_id in self.seen:
            answer = (False, 'expired duplicate request_id')
        elif req.run_id and req.run_id != self.run_id:
            answer = (False, 'run_id mismatch')
        elif not self.run_id:
            answer = (False, 'rover run_id not known yet')
        elif req.command == req.START:
            if self.child:
                answer = (False, 'already recording')
            else:
                try:
                    self.start_recording()
                    answer = (True, str(self.bag))
                except Exception as exc:
                    answer = (False, str(exc))
                    self.state, self.reason = 'FAILED', str(exc)
        elif req.command == req.STOP:
            answer = self.stop_recording()
        else:
            answer = (False, 'unknown bag command')
        if req.request_id:
            self.seen[req.request_id] = req.command
            self.cache.save(req.request_id, answer)
        if self.metadata:
            self.metadata['recorder_requests'].append({'request_id': req.request_id, 'command': req.command,
                'accepted': answer[0], 'reason': answer[1],
                'sent_stamp_ns': req.sent_stamp.sec*1000000000+req.sent_stamp.nanosec})
            if self.bag and self.bag.exists():
                self.write_metadata()
        res.accepted, res.reason = answer
        return res

    def tick(self):
        if self.auto and self.run_id and self.state == 'IDLE':
            self.auto = False
            self.start_recording()
        if self.child:
            if self.bag.exists() and not (self.bag/'run_metadata.json').exists():
                self.write_metadata()
            if self.child.poll() is not None:
                self.stop_recording('recorder exited unexpectedly')
            elif self.bag.exists():
                self.state = 'RECORDING'
                if time.monotonic()-self.started > self.max_sec or self.size() > self.max_bytes:
                    self.stop_recording('recording size/time limit reached')
        msg = RecordingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state, msg.run_id, msg.reason = self.state, self.run_id, self.reason
        msg.bag_path = str(self.bag) if self.bag else ''
        msg.bytes_written = self.size()
        self.status_pub.publish(msg)

    def size(self):
        return sum(p.stat().st_size for p in self.bag.rglob('*') if p.is_file()) if self.bag and self.bag.exists() else 0


def main():
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GcsRecorder()
    try:
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        if node.child:
            node.stop_recording('recorder node shutdown')
        node.destroy_node()
        rclpy.shutdown()
