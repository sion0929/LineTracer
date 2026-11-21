#!/usr/bin/env python3
"""Repeated owned composed launch SIGINT: running camera, normal container exit, empty group."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
from checkpoint_a import Check, WS


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--repetitions',type=int,default=10)
    parser.add_argument('--launch-file')
    parser.add_argument('--label',default='teardown')
    args=parser.parse_args()
    def interrupted(*_): raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupted)
    output=WS/'log_lab/d06'/args.label; output.mkdir(parents=True,exist_ok=True)
    results=[]
    for index in range(args.repetitions):
        os.environ['ROS_DOMAIN_ID']='94'
        os.environ['GZ_PARTITION']=f'd06_teardown_{os.getpid()}_{index}'
        c=Check('sensors')
        ready=False
        try:
            if args.launch_file:
                c.launch=subprocess.Popen(['ros2','launch',args.launch_file],stdout=c.log,
                                          stderr=subprocess.STDOUT,start_new_session=True)
            else: c.start()
            ready=c.wait(lambda:len(c.data['/camera/image_raw'])>=15,30)
        finally:
            c.close()
        log=c.log_path.read_text()
        clean=bool(re.search(r'\[INFO\].*component_container_mt.*process has finished cleanly',log))
        crashes=re.findall(r'component_container_mt.*exit code (-?\d+)',log)
        ok=ready and clean and not crashes and all(c.rows)
        row={'iteration':index+1,'pass':ok,'camera_ready':ready,'normal_exit':clean,'exit_errors':crashes,
             'log':str(output/f'{index+1:02}.log')}
        shutil.copyfile(c.log_path,row['log']); results.append(row)
        print(f'{"PASS" if ok else "FAIL"} | teardown {index+1}/{args.repetitions} | '+json.dumps(row),flush=True)
    (output/'results.json').write_text(json.dumps(results,indent=2))
    count=sum(r['pass'] for r in results)
    print(f'TEARDOWN: {"PASS" if count==args.repetitions else "FAIL"} ({count}/{args.repetitions} clean)',flush=True)
    return 0 if count==args.repetitions else 1


if __name__=='__main__':raise SystemExit(main())
