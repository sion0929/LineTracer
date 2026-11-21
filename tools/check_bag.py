#!/usr/bin/env python3
"""D-13 real recording, shared event analysis, Qt scrubbing and safe pipeline replay."""
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
os.environ['PYTHONUNBUFFERED'] = '1'
from line_follower_interfaces.msg import ObservedGraph
from checkpoint_a import Check, speed, zero, WS
from check_gcs import GcsCheck
from line_follower_interfaces.msg import RecordingStatus, LineObservation, PerceptionStatus, LocalPath, PathStatus
from line_follower_interfaces.srv import BagCommand
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from line_follower_gcs.bag_archive import BagArchive
from line_follower_gcs.gcs_recorder import record_topics, fingerprint


class RecordCheck(GcsCheck):
    def __init__(self, root):
        super().__init__()
        self.root = root
        self.node.set_parameters([Parameter('use_sim_time', value=True)])
        self.statuses = []
        self.node.create_subscription(RecordingStatus, '/gcs/recording_status', self.statuses.append, 100)
        self.bag_client = self.node.create_client(BagCommand, '/gcs/recorder/command')
        self.bag = None

    def start(self):
        self.started = time.monotonic()
        self.launch = subprocess.Popen(['ros2','launch','line_follower_bringup','dev_sim.launch.py',
            'headless:=true','gcs:=true',f'bag_root:={self.root}'],
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def bag_call(self, command, request_id=None, run_id=None):
        req = BagCommand.Request()
        req.request_id = request_id or str(uuid.uuid4())
        req.run_id = run_id if run_id is not None else self.latest('/gcs/snapshot').mission.run_id
        req.command = command
        req.sent_stamp = self.node.get_clock().now().to_msg()
        future = self.bag_client.call_async(req)
        if not self.wait(future.done, 10):
            raise RuntimeError('bag service response deadline')
        return req, future.result()

    def exercise_bag(self):
        if not self.wait(lambda: self.latest('/gcs/snapshot') and self.bag_client.service_is_ready(), 12):
            raise RuntimeError('GCS/recorder not available')
        _, answer = self.bag_call(1)
        self.row('STOP while idle rejected', not answer.accepted, answer.reason)
        _, answer = self.bag_call(0, run_id='wrong-run')
        self.row('recorder wrong run rejected', not answer.accepted, answer.reason)
        request, answer = self.bag_call(0)
        self.row('record START accepted', answer.accepted, answer.reason)
        if not answer.accepted:
            raise RuntimeError('recording did not start')
        self.bag = Path(answer.reason)
        _, duplicate = self.bag_call(0, request_id=request.request_id)
        self.row('duplicate START original reply', duplicate==answer, duplicate.reason)
        _, conflict = self.bag_call(1, request_id=request.request_id)
        self.row('different command ID rejected', not conflict.accepted and 'different command' in conflict.reason, conflict.reason)
        _, active = self.bag_call(0)
        self.row('second START rejected', not active.accepted, active.reason)
        self.row('recording status active', self.wait(lambda: self.statuses and self.statuses[-1].state=='RECORDING', 6),
                 f'state={self.statuses[-1].state if self.statuses else None}')
        if not self.wait(lambda: self.latest('/gcs/snapshot').mission.state==3 and
                         self.data['/odom'] and speed(self.latest('/odom'))[0]>.025, 55):
            raise RuntimeError('no recorded RUNNING motion')
        self.broker('kill', 'recorded KILL accepted', 'accepted')
        self.row('recorded rover STOPPED', self.wait(lambda: self.latest('/mission/state').state==8, 1.5),
                 f'state={self.latest("/mission/state").state}')
        self.spin(1.)
        stopped_at = time.monotonic()
        stop_request, answer = self.bag_call(1)
        self.row('record STOP closes cleanly', answer.accepted, f'{answer.reason}; close={time.monotonic()-stopped_at:.3f}s')
        _, cached_stop = self.bag_call(1, request_id=stop_request.request_id)
        self.row('duplicate STOP original reply', cached_stop==answer, cached_stop.reason)
        _, stopped_duplicate = self.bag_call(1)
        self.row('second STOP rejected', not stopped_duplicate.accepted, stopped_duplicate.reason)
        self.spin(.3)
        self.row('recording status STOPPED', self.statuses[-1].state=='STOPPED', self.statuses[-1].state)
        metadata = json.loads((self.bag/'run_metadata.json').read_text())
        self.row('bag metadata contract', (self.bag/'metadata.yaml').exists() and metadata['closed_cleanly'] and
                 metadata['run_id']==self.latest('/mission/state').run_id and metadata['use_sim_time'] and
                 metadata['stop'] and metadata['code_config_sha256']==fingerprint(WS, Path(metadata['params_file'])),
                 json.dumps(metadata, sort_keys=True))
        archive = BagArchive(self.bag)
        required = set(record_topics())-{'/perception/obstacles'}
        missing = [t for t in required if archive.counts.get(t, 0)==0]
        self.row('expected topics and counts', not missing, f'missing={missing}; counts={json.dumps(archive.counts, sort_keys=True)}')
        states = [e['text'] for e in archive.events if e['kind']=='mission']
        expected = ['STARTUP_WAIT','CHECKING','READY','RUNNING','E_STOP','STOPPED']
        self.row('full mission event sequence', states==expected, str(states))
        self.row('KILL command event', any(e['kind']=='command' and e.get('command')=='kill' and
                 e.get('outcome')=='accepted' for e in archive.events), f'{len(archive.events)} total events')
        for event in archive.events:
            print(f'EVENT {event["time"]:.3f}s {event["kind"]}: {event["text"]}', flush=True)
        report = self.root/'analysis.json'
        analyzer = subprocess.run(['python3','tools/bag_analyzer/run_analyzer.py',str(self.bag),'--json','--output',str(report)],
                                  cwd=WS, capture_output=True, text=True)
        self.row('CLI uses shared extractor', analyzer.returncode==0 and
                 json.loads(report.read_text())['events']==archive.events, f'JSON={report}; exit={analyzer.returncode}; stderr={analyzer.stderr.strip()}')
        size = sum(p.stat().st_size for p in self.root.rglob('*') if p.is_file())
        self.row('bounded recording artifact', size<128*1024*1024, f'bag={self.bag}; total={size/1048576:.2f} MiB; duration={archive.duration:.3f}s')
        ui = subprocess.run(['python3','tools/check_bag_ui.py',str(self.bag)], cwd=WS, capture_output=True, text=True)
        print(ui.stdout, end='', flush=True)
        self.row('real bag offscreen scrub', ui.returncode==0, f'exit={ui.returncode}; stderr={ui.stderr.strip()}')


class ReplayCheck(Check):
    def __init__(self, bag):
        super().__init__('bag-replay')
        self.bag = bag
        self.counts = {}
        self.blocked = []
        self.max_publishers = 0
        for topic, typ in [('/perception/line',LineObservation),('/perception/status',PerceptionStatus),
                           ('/path/local',LocalPath),('/path/status',PathStatus),('/path/memory',ObservedGraph)]:
            self.counts[topic] = 0
            self.node.create_subscription(typ,topic,lambda m,t=topic:self.count(t),100)
        self.node.create_subscription(Twist,'/replay/cmd_vel_blocked',self.blocked.append,1000)

    def count(self, topic):
        self.counts[topic] += 1

    def start(self):
        self.launch = subprocess.Popen(['ros2','launch','line_follower_bringup','replay.launch.py',
                                       f'bag:={self.bag}','rate:=2.0'], stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)

    def exercise(self):
        duration = BagArchive(self.bag).duration/2+3.
        end = time.monotonic()+duration
        while time.monotonic()<end:
            self.spin(.04)
            self.max_publishers = max(self.max_publishers, self.node.count_publishers('/cmd_vel'))
        self.row('replay never publishes cmd_vel', self.max_publishers==0 and not self.data['/cmd_vel'],
                 f'max publishers={self.max_publishers}; received={len(self.data["/cmd_vel"])}')
        self.row('replay command redirected', len(self.blocked)>20 and any(not zero(m) for m in self.blocked),
                 f'/replay/cmd_vel_blocked count={len(self.blocked)}; nonzero={sum(not zero(m) for m in self.blocked)}')
        self.row('perception/path regenerated', all(v>10 for v in self.counts.values()), json.dumps(self.counts,sort_keys=True))
        self.row('replay excludes Gazebo bridge', not {'ros_gz_bridge','camera_bridge'}.intersection(self.node.get_node_names()),
                 f'nodes={sorted(self.node.get_node_names())}')


def main():
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'signal {signum}')
    signal.signal(signal.SIGTERM, interrupted)
    parent = WS/'bags/check-stage2'
    parent.mkdir(parents=True, exist_ok=True)
    for previous in parent.iterdir():
        if previous.is_dir() and (previous/'check_complete').exists():
            shutil.rmtree(previous)
    root = parent/uuid.uuid4().hex[:8]
    root.mkdir(parents=True)
    os.environ['ROS_DOMAIN_ID']='84'
    os.environ['GZ_PARTITION']=f'bag_record_{os.getpid()}'
    check = RecordCheck(root)
    try:
        check.start()
        check.exercise_bag()
    except (Exception, KeyboardInterrupt) as exc:
        check.row('record/analysis completed',False,str(exc))
    finally:
        check.close()
    ok = all(check.rows)
    if ok:
        os.environ['ROS_DOMAIN_ID']='85'
        os.environ['GZ_PARTITION']=f'bag_replay_{os.getpid()}'
        replay = ReplayCheck(check.bag)
        try:
            replay.start()
            replay.exercise()
        except (Exception,KeyboardInterrupt) as exc:
            replay.row('replay completed',False,str(exc))
        finally:
            replay.close()
        ok = all(replay.rows)
    (root/'check_complete').write_text('PASS\n' if ok else 'FAIL\n')
    print(f'D-13 RECORD/REPLAY: {"PASS" if ok else "FAIL"}; artifacts={root}',flush=True)
    return 0 if ok else 1


if __name__=='__main__':
    raise SystemExit(main())
