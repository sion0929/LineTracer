#!/usr/bin/env python3
"""Compare identical slow RUNNING workloads for 20 wall seconds, without an external image reader."""
import json
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time
import yaml

from checkpoint_a import Check, WS
from line_follower_interfaces.msg import LineObservation, PerceptionStatus
from rcl_interfaces.srv import GetParameters


class Measure(Check):
    def __init__(self, composed, params):
        super().__init__('composed' if composed else 'standalone')
        # Do not introduce a 27 MB/s external camera subscriber into the benchmark.
        self.node.destroy_subscription(self.subs.pop(0))
        self.composed, self.params = composed, params
        self.lines, self.statuses = [], []
        self.node.create_subscription(LineObservation, '/perception/line',
                                      lambda m:self.lines.append((time.monotonic(),m)), 1000)
        self.node.create_subscription(PerceptionStatus, '/perception/status',
                                      lambda m:self.statuses.append((time.monotonic(),m)), 1000)

    def start(self):
        self.started=time.monotonic()
        self.launch=subprocess.Popen(['ros2','launch','line_follower_bringup','competition.launch.py',
            'headless:=true',f'compose:={str(self.composed).lower()}',f'params_file:={self.params}'],
            stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)

    def ticks(self,pid):
        fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        return int(fields[11])+int(fields[12])

    def measure(self):
        if not self.wait(lambda:any(m.state==3 for _,m in self.data['/mission/state']) and self.statuses,55):
            raise RuntimeError('not RUNNING')
        self.spin(1.)
        names=['perception_container'] if self.composed else ['camera_bridge','line_detector','path_planner']
        # Launch prefixes use executable names for ros_gz_image, explicit component name for the container.
        pids={name:self.owned_pid('image_bridge' if name=='camera_bridge' else
                                'component_container_mt' if name=='perception_container' else name) for name in names}
        client=self.node.create_client(GetParameters,'/path_planner/get_parameters')
        if not self.wait(client.service_is_ready,3.):
            raise RuntimeError('planner parameters unavailable')
        future=client.call_async(GetParameters.Request(names=['max_speed']))
        if not self.wait(future.done,2.):
            raise RuntimeError('parameter query deadline')
        self.row('benchmark parameter loaded', abs(future.result().values[0].double_value-.001)<1e-9,
                 f'path_planner.max_speed={future.result().values[0].double_value}')
        before={n:self.ticks(pid) for n,pid in pids.items()}
        started=time.monotonic()
        self.spin(20.)
        ended=time.monotonic()
        cpu={n:(self.ticks(pids[n])-v)/os.sysconf('SC_CLK_TCK')/(ended-started)*100
             for n,v in before.items()}
        statuses=[(t,m) for t,m in self.statuses if started<=t<=ended]
        lines=[(t,m) for t,m in self.lines if started<=t<=ended]
        latency=sorted(m.capture_to_publish_ms for _,m in statuses)
        stamps=lambda messages:{(m.header.stamp.sec,m.header.stamp.nanosec) for _,m in messages}
        captures={(m.last_capture_stamp.sec,m.last_capture_stamp.nanosec) for _,m in statuses}
        report={'variant':'composed' if self.composed else 'standalone',
            'wall_sec':ended-started, 'camera_callbacks':len(statuses),
            'camera_seen_hz':len(statuses)/(ended-started), 'line_messages':len(lines),
            'line_hz':len(lines)/(ended-started),'latency_p50_ms':statistics.median(latency),
            'latency_p95_ms':latency[min(len(latency)-1,int(.95*len(latency)))],
            'negative_latency_samples':sum(v<0 for v in latency),'cpu_percent':cpu,
            'cpu_total_percent':sum(cpu.values()),'pids':pids,'max_excursion_m':self.max_excursion,
            'matched_capture_stamps':len(captures & stamps(lines))}
        self.row('measured RUNNING workload',len(statuses)>100 and len(lines)>100,
                 json.dumps(report,sort_keys=True))
        log=self.log_path.read_text()
        if self.composed:
            self.row('actual intra-process callbacks', 'camera callback intra_process=true' in log and
                     'line callback intra_process=true' in log,
                     '\n'.join(line for line in log.splitlines() if 'callback intra_process=' in line))
            components=subprocess.run(['ros2','component','list','--no-daemon','--spin-time','3','/perception_container'],
                                      capture_output=True,text=True,timeout=10)
            self.row('component CLI list',components.returncode==0 and all('/'+n in components.stdout
                     for n in ('camera_bridge','line_detector','path_planner')),
                     f'exit={components.returncode}; stdout={components.stdout.strip()}; stderr={components.stderr.strip()}')
            report['component_list']=components.stdout
        return report


def main():
    os.environ['PYTHONUNBUFFERED']='1'
    def interrupted(signum,frame):
        raise KeyboardInterrupt(f'signal {signum}')
    signal.signal(signal.SIGTERM,interrupted)
    output=WS/'log_lab/components'
    output.mkdir(parents=True,exist_ok=True)
    config=yaml.safe_load((WS/'src/line_follower_bringup/config/competition.yaml').read_text())
    config['path_planner']['ros__parameters']['max_speed']=.001
    config['motion_controller']['ros__parameters']['max_linear']=.001
    params=output/'benchmark.yaml'
    params.write_text(yaml.safe_dump(config))
    reports=[]
    ok=True
    for composed in (False,True):
        os.environ['ROS_DOMAIN_ID']='89' if composed else '88'
        os.environ['GZ_PARTITION']=f'components_{os.getpid()}_{int(composed)}'
        check=Measure(composed,params)
        try:
            check.start()
            reports.append(check.measure())
        except (Exception,KeyboardInterrupt) as exc:
            check.row('measurement completed',False,str(exc))
        finally:
            check.close()
        ok=ok and all(check.rows)
    (output/'measurements.json').write_text(json.dumps(reports,indent=2)+'\n')
    print('COMPONENT MEASUREMENT: '+('PASS' if ok else 'FAIL'),flush=True)
    return 0 if ok else 1


if __name__=='__main__':
    raise SystemExit(main())
