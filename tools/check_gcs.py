#!/usr/bin/env python3
"""D-11/D-12 real headless system acceptance; isolated and self-cleaning."""
import os
os.environ['ROS_DOMAIN_ID'] = '82'
os.environ['GZ_PARTITION'] = f'gcs_acceptance_{os.getpid()}'
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
import math
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time
import uuid
import rclpy
from geometry_msgs.msg import Twist
from composition_interfaces.srv import ListNodes, UnloadNode
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from line_follower_interfaces.msg import (GcsSnapshot, GcsLinkStatus, MissionState,
                                          GcsCommandRequest, GcsCommandEvent)
from line_follower_interfaces.srv import MissionCommand
from checkpoint_a import Check, speed, zero


class GcsCheck(Check):
    def __init__(self):
        super().__init__('gcs')
        self.extra = {}
        for topic, typ in [('/mission/gcs_snapshot', GcsSnapshot), ('/gcs/snapshot', GcsSnapshot),
                           ('/gcs/link', GcsLinkStatus), ('/gcs/command_events', GcsCommandEvent)]:
            self.extra[topic] = []
            self.subs.append(self.node.create_subscription(
                typ, topic, lambda m, t=topic: self.extra[t].append((time.monotonic(), m)), 100))
        self.requests = self.node.create_publisher(GcsCommandRequest, '/gcs/command_requests', 10)
        self.client = self.node.create_client(MissionCommand, '/mission/command')
        self.params = self.node.create_client(SetParameters, '/mission_manager/set_parameters')

    def start(self):
        self.started = time.monotonic()
        self.launch = subprocess.Popen(['ros2', 'launch', 'line_follower_bringup', 'dev_sim.launch.py',
                                        'headless:=true', 'gcs:=true', 'recorder:=false', 'auto_start:=false'],
                                       stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def latest(self, topic):
        values = self.extra.get(topic, self.data.get(topic, []))
        return values[-1][1] if values else None

    def broker(self, command, label, expected):
        msg = GcsCommandRequest()
        msg.request_id = str(uuid.uuid4())
        msg.run_id = self.latest('/gcs/snapshot').mission.run_id
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.command = command
        self.requests.publish(msg)
        found = lambda: next((m for _, m in self.extra['/gcs/command_events'] if m.request_id == msg.request_id), None)
        self.wait(found, 3.5)
        result = found()
        self.row(label, result is not None and result.outcome == expected,
                 f'{result.outcome}: {result.reason}' if result else 'no event')
        return result

    def direct(self, request):
        future = self.client.call_async(request)
        if not self.wait(future.done, 2):
            raise RuntimeError('mission command did not respond')
        return future.result()

    def stop_source(self, name):
        if name in ('line_detector', 'path_planner') and 'perception_container' in self.node.get_node_names():
            listing = self.node.create_client(ListNodes, '/perception_container/_container/list_nodes')
            unload = self.node.create_client(UnloadNode, '/perception_container/_container/unload_node')
            if not self.wait(lambda: listing.service_is_ready() and unload.service_is_ready(), 3.):
                raise RuntimeError('component manager services unavailable')
            future = listing.call_async(ListNodes.Request())
            if not self.wait(future.done, 2.):
                raise RuntimeError('component list deadline')
            result = future.result()
            identifier = result.unique_ids[list(result.full_node_names).index('/'+name)]
            future = unload.call_async(UnloadNode.Request(unique_id=identifier))
            if not self.wait(future.done, 2.) or not future.result().success:
                raise RuntimeError(f'failed to unload component {name}')
        else:
            os.kill(self.owned_pid(name), signal.SIGINT)

    def snapshot_enabled(self, enabled):
        req = SetParameters.Request()
        req.parameters = [Parameter(name='gcs_snapshot_enabled', value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=enabled))]
        future = self.params.call_async(req)
        if not self.wait(future.done, 2) or not all(r.successful for r in future.result().results):
            raise RuntimeError('snapshot toggle failed')

    def exercise(self):
        if not self.wait(lambda: self.latest('/gcs/snapshot') and
                         self.node.count_subscribers('/gcs/command_requests') >= 1 and
                         self.client.service_is_ready() and self.params.service_is_ready(), 15):
            raise RuntimeError('GCS did not connect')
        self.broker('start', 'START before READY rejected', 'rejected')
        req = MissionCommand.Request()
        req.request_id, req.run_id, req.command = str(uuid.uuid4()), 'wrong-run', req.START
        res = self.direct(req)
        self.row('wrong run rejected', not res.accepted and 'run_id' in res.reason, res.reason)
        self.broker('bag_start', 'bag START missing recorder', 'timeout')
        self.broker('bag_stop', 'bag STOP missing recorder', 'timeout')
        sample_start = time.monotonic()
        self.spin(3.)
        samples = [(t, m) for t, m in self.extra['/gcs/snapshot'] if t >= sample_start]
        hz = (len(samples)-1)/(samples[-1][0]-samples[0][0]) if len(samples)>1 else 0
        self.row('GCS snapshot rate', 1.7 <= hz <= 2.3, f'{len(samples)} samples; {hz:.3f} Hz')
        source = {m.header.stamp.sec*10**9+m.header.stamp.nanosec: (t, m)
                  for t, m in self.extra['/mission/gcs_snapshot']}
        delays = [t-source[m.header.stamp.sec*10**9+m.header.stamp.nanosec][0] for t, m in samples
                  if m.header.stamp.sec*10**9+m.header.stamp.nanosec in source]
        equal = all(m.mission == source[m.header.stamp.sec*10**9+m.header.stamp.nanosec][1].mission
                    for _, m in samples if m.header.stamp.sec*10**9+m.header.stamp.nanosec in source)
        self.row('snapshot relay state equality', bool(delays) and equal and max(delays)<.5,
                 f'{len(delays)} matched stamps; max relay={max(delays, default=math.inf):.4f}s')
        snap = self.latest('/gcs/snapshot')
        self.row('odom geometry and provenance', snap.pose_valid and snap.path_valid and
                 snap.header.frame_id == 'odom' and snap.observed_graph.header.frame_id == 'odom' and
                 snap.selected_path.header.frame_id == 'odom' and bool(snap.observed_graph.segments) and
                 bool(snap.selected_path.points),
                 f'graph={len(snap.observed_graph.segments)} path={len(snap.selected_path.points)} '
                 f'sources={sorted({p.source for p in snap.selected_path.points})}')
        # Lose only snapshots while manual START is gated; mission and safety keep running.
        self.snapshot_enabled(False)
        self.spin(.3)
        last_snapshot = self.extra['/gcs/snapshot'][-1][0]
        self.wait(lambda: self.latest('/gcs/link').state == 'LOST', 3.)
        lost = [(t, m) for t, m in self.extra['/gcs/link'] if t >= last_snapshot and m.state == 'LOST']
        delay = lost[0][0]-last_snapshot if lost else math.inf
        states_before = len(self.data['/mission/state'])
        self.spin(.3)
        core = {'mission_manager', 'line_detector', 'path_planner', 'motion_controller', 'safety_guard'}
        self.row('link loss visible and isolated', 1.9 <= delay <= 2.4 and
                 core <= set(self.node.get_node_names()) and len(self.data['/mission/state'])>states_before,
                 f'LOST after {delay:.3f}s; all five rover nodes alive; mission publishes')
        self.snapshot_enabled(True)
        self.row('link recovers', self.wait(lambda: self.latest('/gcs/link').state == 'OK', 1.5),
                 self.latest('/gcs/link').state)
        if not self.wait(lambda: self.latest('/mission/state').state == MissionState.READY, 25):
            raise RuntimeError('manual mission never READY')
        req = MissionCommand.Request()
        req.request_id = str(uuid.uuid4())
        req.run_id = self.latest('/gcs/snapshot').mission.run_id
        req.command = req.START
        first = self.direct(req)
        self.row('START READY manual accepted', first.accepted, first.reason)
        if not self.wait(lambda: self.latest('/mission/state').state == MissionState.RUNNING, 1.):
            raise RuntimeError('accepted START did not execute')
        duplicate = self.direct(req)
        self.row('duplicate returns original', duplicate == first and duplicate.state == MissionState.READY,
                 f'accepted={duplicate.accepted}; cached state={duplicate.state}; same response={duplicate==first}')
        self.broker('start', 'START RUNNING rejected', 'rejected')
        self.row('live RUNNING reaches UI feed', self.wait(lambda: self.latest('/gcs/snapshot').mission.state == MissionState.RUNNING, .6),
                 f'GCS state={self.latest("/gcs/snapshot").mission.state}')
        running_source = next(t for t, m in self.data['/mission/state'] if m.state == MissionState.RUNNING)
        running_display = next(t for t, m in self.extra['/gcs/snapshot'] if m.mission.state == MissionState.RUNNING)
        self.row('RUNNING display within period', running_display-running_source <= .55,
                 f'rover→GCS state delay={running_display-running_source:.4f}s (2 Hz + 50ms transport margin)')
        self.row('real rover moves', self.wait(lambda: self.data['/odom'] and speed(self.latest('/odom'))[0]>.025, .6),
                 f'odom speed={speed(self.latest("/odom"))[0]:.4f}m/s')
        # Wait for an actual Qt-render telemetry line with RUNNING, not just bridge reception.
        # UI logs rendered state on each snapshot, so this needs less than one period.
        self.wait(lambda: re.search(r'UI_RENDER .*state=3', self.log_path.read_text()), .6)
        ui_pid = self.owned_pid('gcs_mission_manager')
        if b'gcs_mission_manager' not in Path(f'/proc/{ui_pid}/cmdline').read_bytes():
            raise RuntimeError('UI PID identity mismatch')
        mission_count = len(self.data['/mission/state'])
        os.kill(ui_pid, signal.SIGINT)
        self.spin(.3)
        core = {'mission_manager','line_detector','path_planner','motion_controller','safety_guard'}
        self.row('UI exit does not stop rover', core <= set(self.node.get_node_names()) and
                 len(self.data['/mission/state'])>mission_count and
                 self.latest('/mission/state').state == MissionState.RUNNING and
                 speed(self.latest('/odom'))[0]>.025,
                 f'UI PID={ui_pid}; rover alive, mission heartbeat, speed={speed(self.latest("/odom"))[0]:.4f}')
        self.broker('kill', 'KILL active accepted', 'accepted')
        self.wait(lambda: self.latest('/mission/state').state == MissionState.STOPPED, 1.)
        seen = {m.state for _, m in self.data['/mission/state']}
        self.row('KILL E_STOP then STOPPED', {MissionState.E_STOP, MissionState.STOPPED} <= seen and
                 not self.latest('/mission/state').drive_enabled, f'states={sorted(seen)}')
        self.spin(.4)
        self.row('KILL final velocity zero', zero(self.latest('/cmd_vel')) and speed(self.latest('/odom'))[0]<.008,
                 f'cmd={self.latest("/cmd_vel").linear.x}; odom={speed(self.latest("/odom"))[0]:.6f}')
        self.broker('kill', 'KILL inactive rejected', 'rejected')
        self.spin(.6)
        transition_states = {event.state for _, snap in self.extra['/gcs/snapshot'] for event in snap.mission_events}
        self.row('timeline retains short states', {MissionState.E_STOP, MissionState.STOPPED} <= transition_states,
                 f'GCS transition events={sorted(transition_states)}')
        log = self.log_path.read_text()
        renders = re.findall(r'UI_RENDER snapshots=(\d+) camera=(\d+) state=(\d+)', log)
        receives = re.findall(r'UI_RECEIVE snapshots=(\d+) camera=(\d+) elapsed=([\d.]+)s snapshot_hz=([\d.]+) camera_hz=([\d.]+)', log)
        self.row('live Qt map/camera render', bool(renders) and any(int(s)>0 and int(c)>0 and state=='3'
                 for s,c,state in renders), f'last render={renders[-1] if renders else None}')
        self.row('live UI reception', bool(receives) and int(receives[-1][0])>10 and int(receives[-1][1])>30,
                 f'snapshots/camera/elapsed/Hz={receives[-1] if receives else None}')
        errors = [line for line in log.splitlines() if 'Traceback' in line or
                  ('process has died' in line and 'gcs_' in line)]
        self.row('GCS processes healthy', not errors, str(errors))
        for executable in ('path_planner','line_detector','safety_guard'):
            self.stop_source(executable)
        self.spin(1.6)
        stale = set(self.latest('/gcs/snapshot').stale_sources)
        expected = {'observed_graph','candidates','path_status','selected_path','safety_status','perception_status'}
        self.row('stopped sources marked stale', expected <= stale and 'pose' not in stale,
                 f'stale_sources={sorted(stale)}; odom remains fresh')


def main():
    # Unit and offscreen widget checks are also included in colcon test.
    result = subprocess.run(['python3', '-m', 'pytest', '-q', 'tools/gcs/test/test_model.py',
                             'tools/gcs/test/test_window.py', 'tools/gcs/test/test_adapter.py'], cwd=Path(__file__).resolve().parents[1])
    if result.returncode:
        return result.returncode
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'signal {signum}')
    signal.signal(signal.SIGTERM, interrupted)
    check = GcsCheck()
    try:
        check.start()
        check.exercise()
    except (Exception, KeyboardInterrupt) as exc:
        check.row('check completed', False, str(exc))
    finally:
        check.close()
    ok = all(check.rows)
    print(f'GCS D-11/D-12: {"PASS" if ok else "FAIL"} ({sum(check.rows)}/{len(check.rows)})', flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
