#!/usr/bin/env python3
"""D-05 dev-only capture/teleport harness and independent mesh ground-truth comparison."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from cv_bridge import CvBridge
from line_follower_interfaces.msg import LineObservation, PerceptionStatus
from sensor_msgs.msg import Image
from checkpoint_a import Check, WS
from d05_groundtruth import load_centerlines, sample_segments, distances, named_poses, project_ground


class Capture(Check):
    def __init__(self):
        super().__init__('d05')
        self.images={}; self.lines=[]; self.statuses=[]
        self.node.create_subscription(Image,'/camera/image_raw',self.image,10)
        self.node.create_subscription(LineObservation,'/perception/line',
                                      lambda m:self.lines.append((time.monotonic(),m)),1000)
        self.node.create_subscription(PerceptionStatus,'/perception/status',
                                      lambda m:self.statuses.append((time.monotonic(),m)),1000)

    def record(self,topic,msg):
        # Teleports are deliberate in this stationary, command-free harness.
        self.data[topic].append((time.monotonic(),(msg.width,msg.height) if topic=='/camera/image_raw' else msg))

    @staticmethod
    def stamp(stamp): return stamp.sec,stamp.nanosec

    def image(self,msg):
        self.images[self.stamp(msg.header.stamp)]=msg
        while len(self.images)>90: del self.images[next(iter(self.images))]

    def start(self):
        self.launch=subprocess.Popen(['ros2','launch',str(WS/'tools/d05_capture.launch.py')],
            stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)

    def start_truth(self):
        # Ground truth is dev-only and is never bridged to the runtime graph.
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.truth=[];self.truth_node=Node()
        def receive(msg):
            for p in msg.pose:
                if p.name=='turtlebot3_waffle_pi':
                    t=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
                    self.truth.append((t,p.position.x,p.position.y,
                        math.atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),
                                   1-2*(p.orientation.y**2+p.orientation.z**2))))
        self.truth_node.subscribe(Pose_V,'/world/line_track/dynamic_pose/info',receive)

    def close(self):
        try:
            if hasattr(self,'truth_node'):
                self.truth_node.unsubscribe('/world/line_track/dynamic_pose/info')
                self.truth_node=None
        finally:super().close()

    def teleport(self,p):
        x,y,z,yaw=p
        request=f'name: "turtlebot3_waffle_pi", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}'
        result=subprocess.run(['gz','service','-s','/world/line_track/set_pose',
            '--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','3000','--req',request],
            capture_output=True,text=True,timeout=5)
        if result.returncode or 'data: true' not in result.stdout:
            raise RuntimeError(f'teleport rejected: {result.stdout} {result.stderr}')
        self.spin(2.)


def world(points,p):
    points=np.asarray(points)
    c,s=math.cos(p[3]),math.sin(p[3])
    return points@np.array([[c,s],[-s,c]])+p[:2]



def score(check,name,p,segments,samples,folder):
    folder.mkdir(parents=True,exist_ok=True)
    begin=time.monotonic(); before=len(check.lines); check.spin(4.)
    elapsed=time.monotonic()-begin
    records=check.lines[before:]
    matched=[m for _,m in records if check.stamp(m.header.stamp) in check.images]
    if not matched: raise RuntimeError('no capture-stamp matched image/observation')
    obs=matched[-1]; image=CvBridge().imgmsg_to_cv2(check.images[check.stamp(obs.header.stamp)],'bgr8')
    capture=obs.header.stamp.sec+obs.header.stamp.nanosec*1e-9
    imu=min((m for _,m in check.data['/imu']),
            key=lambda m:abs(m.header.stamp.sec+m.header.stamp.nanosec*1e-9-capture))
    q=imu.orientation
    pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
    roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
    points=np.array([[q.x,q.y] for seg in obs.segments for q in seg.points]).reshape(-1,2)
    detected=world(points,p)
    error=distances(detected,segments)
    c,s=math.cos(p[3]),math.sin(p[3]); local=(samples[:,:2]-p[:2])@np.array([[c,-s],[s,c]])
    uv=project_ground(local,pitch,roll)
    visible=(local[:,0]>.30)&(local[:,0]<1.19)&(uv[:,0]>=2)&(uv[:,0]<638)&(uv[:,1]>=242)&(uv[:,1]<478)&(samples[:,2]<.005)&(np.linalg.norm(local,axis=1)<=obs.observed_range_max+.005)&(np.linalg.norm(local,axis=1)>=obs.observed_range_min-.005)
    target=samples[visible,:2]
    detected_segments=[]
    for seg in obs.segments:
        pts=world([[q.x,q.y] for q in seg.points],p)
        detected_segments.extend([[a,b] for a,b in zip(pts[:-1],pts[1:])])
    recall=float(np.mean(distances(target,np.pad(np.array(detected_segments),((0,0),(0,0),(0,1))))<.03)) if detected_segments and len(target) else 0.
    landmarks={'start_t':segments[0,1,:2], 'corner':segments[99,1,:2],
               'crossing':np.array([segments[320,0,0],segments[143,0,1]]),
               'finish_t':segments[-1].mean(axis=0)[:2]}
    required={'start_t':3,'corner':2,'crossing':4,'finish_t':3}.get(name,0)
    candidates=[{'world':world([[j.x,j.y]],p)[0].tolist(),'legs':len(j.segment_ids)} for j in obs.junctions]
    candidate_ok=not required or any(j['legs']==required and np.linalg.norm(np.array(j['world'])-landmarks[name])<.05 for j in candidates)
    timing=[m.postprocess_ms for t,m in check.statuses if t>=begin]
    metrics={'pose':list(map(float,p)),'frames':len(records),'line_hz':len(records)/elapsed,
             'line_found_fraction':sum(m.line_found for _,m in records)/max(1,len(records)),
             'projection_valid_fraction':sum(m.projection_valid for _,m in records)/max(1,len(records)),
             'median_m':float(np.median(error)) if len(error) else None,
             'p95_m':float(np.percentile(error,95)) if len(error) else None,
             'max_m':float(np.max(error)) if len(error) else None,'recall':recall,
             'segments':len(obs.segments),'candidates':candidates,'candidate_ok':candidate_ok,
             'range_min':obs.observed_range_min,'range_max':obs.observed_range_max,
             'quality':obs.quality,'projection_valid':obs.projection_valid,
             'processing_p50_ms':float(np.median(timing)) if timing else None,
             'processing_p95_ms':float(np.percentile(timing,95)) if timing else None,
             'capture_stamp':list(check.stamp(obs.header.stamp)),
             'imu_pitch':math.asin(max(-1.,min(1.,2*(check.data['/imu'][-1][1].orientation.w*check.data['/imu'][-1][1].orientation.y-check.data['/imu'][-1][1].orientation.z*check.data['/imu'][-1][1].orientation.x))))}
    colors=[(0,200,0),(255,160,0),(0,100,255),(255,0,220)]
    for i,seg in enumerate(obs.segments):
        uv=project_ground([[q.x,q.y] for q in seg.points],pitch,roll).astype(np.int32)
        cv2.polylines(image,[uv],False,colors[i%len(colors)],2,cv2.LINE_AA)
    for j in obs.junctions:
        uv=project_ground([[j.x,j.y]],pitch,roll)[0].astype(int)
        cv2.circle(image,tuple(uv),6,(0,255,255),2)
        cv2.putText(image,str(len(j.segment_ids)),tuple(uv+8),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1)
    cv2.imwrite(str(folder/f'{name}_overlay.png'),image)
    fig,ax=plt.subplots(figsize=(6,6))
    for seg in segments: ax.plot(seg[:,0],seg[:,1],color='.65',linewidth=2)
    if len(target): ax.scatter(target[:,0],target[:,1],s=5,c='k',label='visible mesh centerline')
    for i,seg in enumerate(obs.segments):
        pts=world([[q.x,q.y] for q in seg.points],p)
        ax.plot(pts[:,0],pts[:,1],'.-',label=f'detected {i}',markersize=3)
    ax.plot(p[0],p[1],'r^'); ax.arrow(p[0],p[1],.1*c,.1*s,width=.008,color='r')
    ax.set_xlim(p[0]-.95,p[0]+1.25); ax.set_ylim(p[1]-.95,p[1]+1.25)
    ax.set_aspect('equal'); ax.grid(); ax.set_title(name); ax.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(folder/f'{name}_topdown.png',dpi=110); plt.close(fig)
    ok=bool(len(error) and np.median(error)<.025 and np.percentile(error,95)<.045 and np.max(error)<.08 and recall>=.80 and candidate_ok and obs.projection_valid and metrics['line_found_fraction']==1. and metrics['projection_valid_fraction']==1.)
    check.row(name,ok if name!='hill_entry' else True,('INFO hill, nongated; ' if name=='hill_entry' else '')+json.dumps(metrics,sort_keys=True))
    metrics['flat_gate_pass']=ok if name!='hill_entry' else None
    return metrics


def main():
    os.environ['ROS_DOMAIN_ID']='92'; os.environ['GZ_PARTITION']=f'd05_{os.getpid()}'
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    folder=WS/'docs/d05'; folder.mkdir(parents=True,exist_ok=True)
    check=Capture(); report={}
    try:
        print('Flat gates: median < 0.025 m; p95 < 0.045 m; max < 0.080 m; recall within reported radial range >= 0.80 at 0.030 m; landmark within 0.050 m with exact leg count; projection valid',flush=True)
        check.start()
        if not check.wait(lambda:len(check.lines)>30 and check.images,35): raise RuntimeError('perception startup timeout')
        check.row('stationary capture graph',not any(n in check.node.get_node_names() for n in ('safety_guard','motion_controller','mission_manager')) and
                  not check.node.get_publishers_info_by_topic('/cmd_vel'),'no command publishers or mission/control nodes')
        segments=load_centerlines(WS); samples=sample_segments(segments)
        for name,p in named_poses(segments).items():
            check.teleport(p)
            report[name]=score(check,name,p,segments,samples,folder)
    except (Exception,KeyboardInterrupt) as exc:
        check.row('D-05 completed',False,str(exc))
    finally:
        check.close()
    (folder/'measurements.json').write_text(json.dumps(report,indent=2)+'\n')
    ok=all(check.rows)
    print('D-05: '+('PASS' if ok else 'FAIL'),flush=True)
    return 0 if ok else 1


if __name__=='__main__': raise SystemExit(main())
