#!/usr/bin/env python3
"""Dev-only simultaneous 20-second C++, Python CLI and rosbag camera observers."""
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import yaml
import rosbag2_py
from checkpoint_a import Check, WS


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    def interrupted(*_):
        raise KeyboardInterrupt('termination requested')
    signal.signal(signal.SIGTERM,interrupted)
    folder = WS/'log_lab/d05-camera'
    folder.mkdir(parents=True, exist_ok=True)
    subprocess.run(['cmake','-S',str(WS/'tools/camera_probe'),'-B',str(folder/'build')],check=True)
    subprocess.run(['cmake','--build',str(folder/'build'),'-j2'],check=True)
    reports = []
    ok=True
    for composed in ((True,) if os.environ.get('RATE_COMPOSED_ONLY') else (False, True)):
        os.environ['ROS_DOMAIN_ID']='91' if composed else '90'
        os.environ['GZ_PARTITION']=f'd05_rate_{os.getpid()}_{int(composed)}'
        check=Check('sensors')
        children=[]
        variant='composed' if composed else 'standalone'
        bag=WS/'bags'/f'camera-rate-{variant}'
        bag.parent.mkdir(parents=True,exist_ok=True)
        if bag.exists():
            shutil.rmtree(bag)
        try:
            config=yaml.safe_load((WS/'src/line_follower_bringup/config/competition.yaml').read_text())
            config['path_planner']['ros__parameters']['max_speed']=0.
            params=folder/f'{variant}.yaml'
            params.write_text(yaml.safe_dump(config))
            check.launch=subprocess.Popen(['ros2','launch','line_follower_bringup','competition.launch.py',
                'headless:=true',f'compose:={str(composed).lower()}',f'params_file:={params}'],
                stdout=check.log,stderr=subprocess.STDOUT,start_new_session=True)
            if not check.wait(lambda: len(check.data['/camera/image_raw'])>30,30):
                raise RuntimeError('camera missing')
            logs=[]
            for name,args in [('bag',['ros2','bag','record','-s','mcap','--storage-preset-profile','zstd_fast','-o',str(bag),'/camera/image_raw','/perception/status']),
                              ('hz',['ros2','topic','hz','/camera/image_raw','--wall-time']),
                              ('cpp',[str(folder/'build/camera_probe')])]:
                log=(folder/f'{variant}-{name}.log').open('w')
                logs.append(log)
                children.append(subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
                                                 env={**os.environ,'PYTHONUNBUFFERED':'1'}))
                if name=='bag':
                    if not check.wait(lambda:any('rosbag' in e.node_name for e in
                            check.node.get_subscriptions_info_by_topic('/camera/image_raw')),8.):
                        raise RuntimeError('recorder discovery deadline')
                    check.spin(.5)
            start=time.monotonic()
            before=len(check.data['/camera/image_raw'])
            check.wait(lambda: children[-1].poll() is not None,28)
            duration=time.monotonic()-start
            for p in children: stop(p)
            for log in logs: log.close()
            metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
            counts={t['topic_metadata']['name']:t['message_count'] for t in metadata['topics_with_message_count']}
            seconds=metadata['duration']['nanoseconds']/1e9
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),
                        rosbag2_py.ConverterOptions('',''))
            stamps={t:[] for t in counts}
            while reader.has_next():
                topic,_,stamp=reader.read_next()
                stamps[topic].append(stamp)
            del reader
            intervals={t:(values[-1]-values[0])/1e9 for t,values in stamps.items() if len(values)>1}
            rates={t:(counts[t]-1)/interval for t,interval in intervals.items() if interval>0}
            report={'variant':variant,'bag_duration_sec':seconds,'bag_counts':counts,
                    'bag_hz':rates,'topic_interval_sec':intervals,
                    'publisher_qos':[{'node':e.node_name,'reliability':e.qos_profile.reliability.name,
                                     'durability':e.qos_profile.durability.name}
                                     for e in check.node.get_publishers_info_by_topic('/camera/image_raw')],
                    'python_reliable_check_hz':(len(check.data['/camera/image_raw'])-before)/duration,
                    'cpp':(folder/f'{variant}-cpp.log').read_text(),
                    'topic_hz':(folder/f'{variant}-hz.log').read_text()}
            print(report,flush=True)
            reports.append(report)
            check.row('C++ observer completes',children[-1].returncode==0,report['cpp'].strip())
            check.row('reliable bag receives camera',rates.get('/camera/image_raw',0)>27. and intervals.get('/camera/image_raw',0)>=19.,
                      f"{counts.get('/camera/image_raw',0)} frames; topic interval={intervals.get('/camera/image_raw',0):.3f}s; rate={rates.get('/camera/image_raw',0):.3f}Hz; whole bag={seconds:.3f}s")
        finally:
            for p in children: stop(p)
            check.close()
            ok=ok and all(check.rows)
    import json
    (folder/'results.json').write_text(json.dumps(reports,indent=2))
    print('CAMERA RATE: '+('PASS' if ok else 'FAIL'),flush=True)
    return 0 if ok else 1


if __name__=='__main__':
    raise SystemExit(main())
