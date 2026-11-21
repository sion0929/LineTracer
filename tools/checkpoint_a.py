#!/usr/bin/env python3
"""Dev-only headless acceptance checks; never loaded by a competition launch.

Run inside the lab with ROS and install_lab sourced. Each invocation owns its
launch process group, ROS domain and Gazebo partition. All deadlines use wall time.
"""
import argparse
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time

from checkpoint_metrics import motion_ok, pose_delta, sample_rate
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu
from line_follower_interfaces.msg import MissionState, SafetyStatus
import yaml

WS = Path(__file__).resolve().parents[1]


def pose(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def speed(msg):
    t = msg.twist.twist
    return math.hypot(t.linear.x, t.linear.y), abs(t.angular.z)


def zero(msg):
    return all(getattr(vec, axis) == 0 for vec in (msg.linear, msg.angular) for axis in ('x', 'y', 'z'))


class Check:
    def __init__(self, mode):
        self.mode = mode
        self.rows = []
        self.data = {}
        self.launch = None
        self.pub = None
        self.origin = None
        self.max_excursion = 0.
        self.log_path = Path(tempfile.mkdtemp(prefix=f'checkpoint-{mode}-')) / 'launch.log'
        self.log = self.log_path.open('w')
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('checkpoint_a_' + mode.replace('-', '_'))
        self.subs = []
        for topic, msg_type in [('/camera/image_raw', Image), ('/camera/camera_info', CameraInfo),
                                ('/imu', Imu), ('/odom', Odometry), ('/clock', Clock),
                                ('/cmd_vel', Twist), ('/mission/state', MissionState),
                                ('/safety/status', SafetyStatus)]:
            self.data[topic] = []
            self.subs.append(self.node.create_subscription(
                msg_type, topic, lambda m, t=topic: self.record(t, m),
                1000 if topic in ('/cmd_vel', '/mission/state', '/safety/status')
                else QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
                if topic=='/camera/image_raw' else qos_profile_sensor_data))
        if mode == 'motion':
            self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def record(self, topic, msg):
        # Retain image metadata rather than several gigabytes of pixels.
        value = (msg.width, msg.height) if topic == '/camera/image_raw' else msg
        self.data[topic].append((time.monotonic(), value))
        if topic == '/odom':
            p = pose(msg)
            if self.origin is None:
                self.origin = p
            self.max_excursion = max(self.max_excursion, math.hypot(p[0]-self.origin[0], p[1]-self.origin[1]))
            if self.max_excursion > .24:
                raise RuntimeError(f'excursion safety limit exceeded: {self.max_excursion:.4f} m')

    def row(self, label, ok, detail):
        self.rows.append(bool(ok))
        print(f'{"PASS" if ok else "FAIL":4} | {label:30} | {detail}', flush=True)

    def wait(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.02)
            if self.launch and self.launch.poll() is not None:
                raise RuntimeError(f'launch exited {self.launch.returncode}')
            if predicate():
                return True
        return False

    def spin(self, duration):
        self.wait(lambda: False, duration)

    def start(self):
        profile = 'sim' if self.mode == 'motion' else 'competition'
        self.started = time.monotonic()
        args = ['ros2', 'launch', 'line_follower_bringup', profile + '.launch.py', 'headless:=true']
        if self.mode == 'sensors':
            config = yaml.safe_load((WS/'src/line_follower_bringup/config/competition.yaml').read_text())
            config['path_planner']['ros__parameters']['max_speed'] = 0.0
            params = self.log_path.parent/'sensor_check.yaml'
            params.write_text(yaml.safe_dump(config))
            args.append(f'params_file:={params}')
        self.launch = subprocess.Popen(
            args,
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def command(self, v=0., w=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub.publish(msg)

    def drive(self, v, w, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.command(v, w)
            self.spin(.04)

    def sensors(self):
        topics = ['/camera/image_raw', '/camera/camera_info', '/imu', '/odom', '/clock']
        self.wait(lambda: all(self.data[t] for t in topics), 45)
        start = time.monotonic()
        self.spin(5)
        types = dict(self.node.get_topic_names_and_types())
        expected = ['sensor_msgs/msg/Image', 'sensor_msgs/msg/CameraInfo', 'sensor_msgs/msg/Imu',
                    'nav_msgs/msg/Odometry', 'rosgraph_msgs/msg/Clock']
        for topic, typ in zip(topics, expected):
            samples = [x for x in self.data[topic] if x[0] >= start]
            self.row(topic, bool(samples) and types.get(topic) == [typ],
                     f'{len(samples)} messages/5s; {types.get(topic)}')
        images = [x for x in self.data[topics[0]] if x[0] >= start]
        hz = sample_rate([t for t, _ in images])
        self.row('camera dimensions and rate', bool(images) and all(m == (640, 480) for _, m in images)
                 and hz >= 5, f'640x480; measured {hz:.2f} Hz wall (world target 30 Hz)')
        subs = self.node.get_subscriptions_info_by_topic('/cmd_vel')
        self.row('velocity bridge subscriber', any(s.node_name == 'ros_gz_bridge' and
                 s.topic_type == 'geometry_msgs/msg/Twist' for s in subs),
                 str([(s.node_name, s.topic_type) for s in subs]))
        pubs = self.node.get_publishers_info_by_topic('/camera/image_raw')
        self.row('camera bridge publisher QoS', bool(pubs) and
                 all(p.node_name == 'camera_bridge' and p.qos_profile.durability.name == 'VOLATILE' and
                     p.qos_profile.history.name in ('KEEP_LAST', 'UNKNOWN') for p in pubs),
                 str([(p.node_name, p.qos_profile.durability.name, p.qos_profile.history.name) for p in pubs]))
        listing = subprocess.run(['ros2', 'component', 'list', '--no-daemon', '--spin-time', '3'],
                                 capture_output=True, text=True, timeout=8)
        self.row('competition component list', listing.returncode == 0 and all(
            '/'+name in listing.stdout for name in ('perception_container','camera_bridge','line_detector','path_planner')),
            f'exit={listing.returncode}; {listing.stdout.strip()}; {listing.stderr.strip()}')
        forbidden = '/' + 'world/' 
        interfaces = [n for n, _ in self.node.get_topic_names_and_types() +
                      self.node.get_service_names_and_types() if n.startswith(forbidden)]
        config = (WS / 'src/line_follower_bringup/config/ros_gz_bridge.yaml').read_text()
        self.row('no scoped bridge interfaces', not interfaces and forbidden not in config,
                 f'ROS graph hits={interfaces}; bridge config scanned')
        assets = subprocess.run(['sha256sum', '-c', 'SHA256SUMS'],
                                cwd=WS / 'src/line_follower_bringup', capture_output=True, text=True)
        self.row('provided asset SHA256', assets.returncode == 0,
                 f'{assets.stdout.count(": OK")} files verified; {assets.stderr.strip()}')

    def stop_check(self, label):
        start = time.monotonic()
        self.drive(0, 0, 1.)
        stopped = [(t, m) for t, m in self.data['/odom'] if t >= start and
                   speed(m)[0] < .008 and speed(m)[1] < .02]
        settled = pose(self.data['/odom'][-1][1])
        hold_start = time.monotonic()
        self.drive(0, 0, 2.)
        hold = [m for t, m in self.data['/odom'] if t >= hold_start]
        drift = max((math.hypot(pose(m)[0]-settled[0], pose(m)[1]-settled[1]) for m in hold), default=math.inf)
        yaw = max((abs(pose_delta(settled, pose(m))[2]) for m in hold), default=math.inf)
        max_v = max((speed(m)[0] for m in hold), default=math.inf)
        max_w = max((speed(m)[1] for m in hold), default=math.inf)
        delay = stopped[0][0]-start if stopped else math.inf
        self.row(label, bool(stopped) and drift < .005 and yaw < .02 and max_v < .008 and max_w < .02,
                 f'stop={delay:.3f}s; 2s drift={drift:.5f}m yaw={yaw:.5f}; speed={max_v:.5f}/{max_w:.5f}')

    def motion(self):
        if not self.wait(lambda: bool(self.data['/odom']) and
                         any(s.node_name == 'ros_gz_bridge' for s in
                             self.node.get_subscriptions_info_by_topic('/cmd_vel')), 45):
            raise RuntimeError('odometry/bridge unavailable')
        self.drive(0, 0, .5)
        for name, v, w in [('forward', .06, 0), ('backward', -.06, 0),
                           ('left turn', .04, .5), ('right turn', .04, -.5),
                           ('rotate left', 0, .6), ('rotate right', 0, -.6)]:
            start = pose(self.data['/odom'][-1][1])
            index = len(self.data['/odom'])
            self.drive(v, w, 1.2)
            delta = pose_delta(start, pose(self.data['/odom'][-1][1]))
            fresh = self.data['/odom'][index:]
            peak_v = max((speed(m)[0] for _, m in fresh), default=0)
            peak_w = max((speed(m)[1] for _, m in fresh), default=0)
            twist_ok = (not v or abs(v) * .35 < peak_v < abs(v) * 1.8) and \
                (not w or abs(w) * .35 < peak_w < abs(w) * 1.8)
            self.row(name, bool(fresh) and twist_ok and motion_ok(v, w, 1.2, delta),
                     f'dx={delta[0]:+.4f}m dy={delta[1]:+.4f}m yaw={delta[2]:+.4f}rad; peak={peak_v:.3f}/{peak_w:.3f}')
            self.stop_check('stop after ' + name)
        self.row('bounded excursion', self.max_excursion < .25, f'max from spawn odom={self.max_excursion:.4f}m')

    def owned_pid(self, name):
        pattern = rf'\[{re.escape(name)}-\d+\]: process started with pid \[(\d+)\]'
        found = lambda: re.search(pattern, self.log_path.read_text())
        if not self.wait(found, 3.):
            raise RuntimeError(f'launch did not report {name} PID')
        pid = int(found().group(1))
        if os.getpgid(pid) != self.launch.pid:
            raise RuntimeError(f'{name} PID not in owned launch process group')
        return pid

    def guard_exit(self):
        self.start_stop()

    def start_stop(self):
        if not self.wait(lambda: any(m.state == MissionState.RUNNING for _, m in
                                    self.data['/mission/state']), 55):
            raise RuntimeError('mission never RUNNING')
        running_t, running = next((t, m) for t, m in self.data['/mission/state']
                                  if m.state == MissionState.RUNNING)
        # Source stamps order events independently of DDS callback scheduling.
        running_stamp = running.header.stamp.sec + running.header.stamp.nanosec * 1e-9
        before = [m for t, m in self.data['/cmd_vel'] if t < running_t]
        odoms = [m for t, m in self.data['/odom'] if
                 m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 < running_stamp]
        drift = max((math.hypot(pose(m)[0]-self.origin[0], pose(m)[1]-self.origin[1])
                     for m in odoms), default=math.inf)
        self.row('20 second startup gate', running.elapsed_since_start >= 20 and
                 running_t-self.started >= 20, f'elapsed={running.elapsed_since_start:.3f}s; launch wall={running_t-self.started:.3f}s')
        self.row('all pre-RUNNING commands zero', bool(before) and all(zero(m) for m in before),
                 f'{len(before)} samples; nonzero={sum(not zero(m) for m in before)}')
        self.row('pre-RUNNING odom stationary', bool(odoms) and drift < .01,
                 f'{len(odoms)} samples; displacement={drift:.6f}m')
        moving = self.wait(lambda: self.data['/odom'] and speed(self.data['/odom'][-1][1])[0] > .025
                           and self.data['/cmd_vel'][-1][1].linear.x > .025, 2)
        self.row('perception pipeline drives', moving, f'odom speed={speed(self.data["/odom"][-1][1])[0]:.4f}m/s')
        if not moving:
            raise RuntimeError('no motion; cannot test interruption')
        if self.mode == 'guard-exit':
            pid = self.owned_pid('safety_guard')
            interrupted = time.monotonic()
            os.kill(pid, signal.SIGINT)
            self.spin(2.)
            cmds = [(t, m) for t, m in self.data['/cmd_vel'] if t >= interrupted]
            self.row('safety_guard exit leaves zero', bool(cmds) and zero(cmds[-1][1]) and
                     not self.node.get_publishers_info_by_topic('/cmd_vel'),
                     f'PID={pid}; post-exit messages={len(cmds)}; last_zero={bool(cmds) and zero(cmds[-1][1])}')
            odoms = [(t, m) for t, m in self.data['/odom'] if t >= interrupted]
            still = [t for t, m in odoms if speed(m)[0]<.008 and speed(m)[1]<.02]
            delay = still[0]-interrupted if still else math.inf
            tail = [m for t, m in odoms if t >= interrupted+1.5]
            self.row('safety_guard exit physical stop', delay <= 1.5 and bool(tail) and
                     all(speed(m)[0]<.008 and speed(m)[1]<.02 for m in tail),
                     f'stop after {delay:.3f}s; final speed={speed(odoms[-1][1]) if odoms else None}')
            return
        pid = self.owned_pid('motion_controller')
        config = yaml.safe_load((WS / 'src/line_follower_bringup/config/competition.yaml').read_text())
        limits = config['safety_guard']['ros__parameters']
        bound = limits['candidate_timeout_sec'] + 2/limits['control_rate_hz'] + .06
        interrupted = time.monotonic()
        os.kill(pid, signal.SIGINT)
        self.spin(2.2)
        cmds = [(t, m) for t, m in self.data['/cmd_vel'] if t >= interrupted]
        stops = [t for t, m in cmds if zero(m)]
        delay = stops[0]-interrupted if stops else math.inf
        self.row('controller loss command stop', delay <= bound and bool(stops) and
                 all(zero(m) for t, m in cmds if t >= stops[0]),
                 f'PID={pid}; zero after {delay:.3f}s; allowed={bound:.3f}s')
        odoms = [(t, m) for t, m in self.data['/odom'] if t >= interrupted]
        still = [t for t, m in odoms if speed(m)[0] < .008 and speed(m)[1] < .02]
        delay = still[0]-interrupted if still else math.inf
        tail = [m for t, m in odoms if t >= interrupted+1.5]
        self.row('controller loss physical stop', delay <= 1.5 and bool(tail) and
                 all(speed(m)[0] < .008 and speed(m)[1] < .02 for m in tail),
                 f'stop after {delay:.3f}s; final speed={speed(odoms[-1][1]) if odoms else None}')
        reasons = {m.stop_reason for t, m in self.data['/safety/status'] if t >= interrupted}
        self.row('safety explains stop', 'command candidate stale' in reasons, str(sorted(reasons)))

    def close(self):
        if self.pub:
            # Publish even on exceptions and Ctrl-C, before tearing down the bridge.
            for _ in range(10):
                self.command()
                time.sleep(.02)
        if self.launch:
            try:
                os.killpg(self.launch.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.launch.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pass
            # A child may outlive the launch parent; clean the owned group too.
            try:
                os.killpg(self.launch.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.launch.wait()
            # Give container init time to reap exited children after the final signal.
            def live_group():
                alive = []
                for proc in Path('/proc').glob('[0-9]*/stat'):
                    try:
                        fields = proc.read_text().rsplit(')', 1)[1].split()
                        if int(fields[2]) == self.launch.pid:
                            alive.append(proc.parent.name)
                    except (OSError, ValueError, IndexError):
                        continue
                return alive
            deadline = time.monotonic()+1.
            alive = live_group()
            while alive and time.monotonic()<deadline:
                time.sleep(.1)
                alive = live_group()
            self.row('owned process group cleaned', not alive, f'live PIDs={alive}')
        self.log.close()
        self.node.destroy_node()
        rclpy.shutdown()
        print(f'Launch log: {self.log_path}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['sensors', 'motion', 'start-stop', 'guard-exit'])
    args = parser.parse_args()
    os.environ['PYTHONUNBUFFERED'] = '1'
    os.environ['ROS_DOMAIN_ID'] = '79'
    os.environ['GZ_PARTITION'] = f'checkpoint_a_{args.mode}_{os.getpid()}'
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'signal {signum}')
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    check = Check(args.mode)
    try:
        check.start()
        getattr(check, args.mode.replace('-', '_'))()
    except (Exception, KeyboardInterrupt) as exc:
        check.row('check completed', False, str(exc) or 'interrupted')
    finally:
        check.close()
    ok = bool(check.rows) and all(check.rows)
    print(f'{args.mode}: {"PASS" if ok else "FAIL"} ({sum(check.rows)}/{len(check.rows)} checks)', flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
