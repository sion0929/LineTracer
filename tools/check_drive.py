#!/usr/bin/env python3
"""Autonomous official-track observer; oracle and simulator truth stay in tools."""
import cv2
from cv_bridge import CvBridge
from collections import deque
import argparse,gc,json,math,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from std_srvs.srv import Trigger
from line_follower_interfaces.msg import PathStatus,LocalPath,PathCandidates,MissionState
from line_follower_interfaces.srv import MissionCommand
from checkpoint_a import WS
from check_d05 import Capture
from d05_groundtruth import load_centerlines


def ordered(segments):
    points=[segments[0,1,:2]];unused=set(range(len(segments)))-{1,len(segments)-1};direction=np.array([1.,0.])
    while unused:
        choices=[]
        for k in unused:
            p=segments[k,:,:2];i=int(np.argmin(np.linalg.norm(p-points[-1],axis=1)))
            gap=np.linalg.norm(p[i]-points[-1]);d=p[1-i]-p[i];d/=np.linalg.norm(d)
            if gap<.015:choices.append((float(direction@d),k,i,d,gap))
        assert choices,'mesh connectivity discontinuity'
        nearest=min(c[4] for c in choices);choices=[c for c in choices if c[4]<=nearest+.0001]
        _,k,i,d,_=max(choices,key=lambda x:x[0]);unused.remove(k)
        if np.linalg.norm(segments[k,i,:2]-points[-1])>.0001:points.append(segments[k,i,:2])
        points.append(segments[k,1-i,:2]);direction=d
    p=np.array(points);length=np.linalg.norm(np.diff(p,axis=0),axis=1)
    return p,np.r_[0,np.cumsum(length)]



class Drive(Capture):
    def __init__(self,name,full):
        super().__init__();self.name=name;self.full=full;self.states=[];self.paths=deque(maxlen=30);self.choices=[];self.lines=deque(maxlen=300)
        for typ,topic,target in [(PathStatus,'/path/status',self.states),(LocalPath,'/path/local',self.paths),(PathCandidates,'/path/candidates',self.choices)]:
            self.node.create_subscription(typ,topic,lambda m,t=target:t.append(m),100)
    def start(self):
        command=['ros2','launch','line_follower_bringup','competition.launch.py','headless:=true'] if self.full else ['ros2','launch',str(WS/'tools/drive_capture.launch.py')]
        self.launch=subprocess.Popen(command,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True,env=dict(os.environ,LF_PROJECTION_DEBUG='1'));self.start_truth()
    def service(self,typ,name,request):
        client=self.node.create_client(typ,name);assert self.wait(client.service_is_ready,5),name
        future=client.call_async(request);assert self.wait(future.done,5),name+' timeout'
        return future.result()
    def run(self,segments,limit,stall_seconds=25.):
        gt,arc=ordered(segments);self.row('ordered mesh oracle',True,f'length={arc[-1]:.3f}m; strips={len(gt)-1}; self crossing retains order')
        assert self.wait(lambda:self.truth and self.states and self.data['/mission/state'],35),'startup timeout'
        if self.full:
            begin=0.;goal=arc[-1]-.12
        else:
            if self.name=='hill':
                point=gt[0];heading=0.;begin=0.;goal=1.90
            elif self.name=='corner':
                vertex=segments[99,1,:2];k=int(np.argmin(np.linalg.norm(gt-vertex,axis=1)))
                previous=max(0,int(np.searchsorted(arc,arc[k]-.2)));direction=vertex-gt[previous];direction/=np.linalg.norm(direction)
                point=vertex-direction*.55;heading=math.atan2(direction[1],direction[0])
                begin=float(arc[k]-.55);goal=begin+.85
            elif self.name=='curve':
                point=segments[12,0,:2]-[.48,0];heading=0.
                begin=float(arc[np.argmin(np.linalg.norm(gt-point,axis=1))]);goal=begin+.90
            elif self.name in ('left_bend','lower_loop'):
                begin,goal=(6.55,7.75) if self.name=='left_bend' else (10.9,12.55)
                k=max(0,int(np.searchsorted(arc,begin))-1);direction=gt[k+1]-gt[k]
                point=gt[k]+direction*((begin-arc[k])/(arc[k+1]-arc[k]));heading=math.atan2(direction[1],direction[0])
            elif self.name=='crossing_east':
                crossing=np.array([segments[320,0,0],segments[143,0,1]])
                point=crossing-[.65,0];heading=0.
                candidates=[i for i in range(len(gt)-1) if np.linalg.norm(gt[i]-point)<.4 and (gt[i+1]-gt[i])[0]>.01]
                begin=float(arc[min(candidates,key=lambda i:np.linalg.norm(gt[i]-point))]);goal=begin+1.65
            else:
                point=np.array([segments[320,0,0],segments[143,0,1]-.55]);heading=math.pi/2
                # Use heading to choose the incoming ordered crossing leg.
                candidates=[i for i in range(len(gt)-1) if np.linalg.norm(gt[i]-point)<.4 and (gt[i+1]-gt[i])[1]>.01]
                begin=float(arc[min(candidates,key=lambda i:np.linalg.norm(gt[i]-point))]);goal=begin+.80
            self.teleport((*point,.01,heading))
            self.service(Trigger,'/path/reset_memory',Trigger.Request());self.spin(.7)
            assert self.wait(lambda:self.data['/mission/state'][-1][1].state==MissionState.READY,8),'not READY'
            mission=self.data['/mission/state'][-1][1]
            response=self.service(MissionCommand,'/mission/command',MissionCommand.Request(request_id=self.name+'-start',run_id=mission.run_id,command=0))
            assert response.accepted,response.reason
        assert self.wait(lambda:any(m.state==MissionState.RUNNING for _,m in self.data['/mission/state']),35),'never RUNNING'
        if self.full:
            running=next(m for _,m in self.data['/mission/state'] if m.state==MissionState.RUNNING)
            self.row('real competition startup gate',running.elapsed_since_start>=20.,f'elapsed={running.elapsed_since_start:.3f}s; default competition arguments')
        last_heartbeat=-10.;start=time.monotonic();initial_stamp=self.truth[-1][0];progress=begin;last_progress=start;off_since=None;history=[];reason='time limit';complete=False
        bagdir=WS/'bags/drive'/f'{self.name}-{os.getpid()}';bagdir.parent.mkdir(parents=True,exist_ok=True)
        self.recorder=subprocess.Popen(['ros2','bag','record','-o',str(bagdir),'/clock','/imu','/localization/estimate','/odom','/mission/state','/perception/line','/path/local','/path/status','/path/candidates','/path/memory','/path/observed_graph','/control/cmd_vel_candidate','/cmd_vel','/safety/status'],stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        while time.monotonic()-start<limit:
            self.spin(.08);q=self.truth[-1];position=np.array(q[1:3])
            a=gt[:-1];delta=np.diff(gt,axis=0);t=np.clip(((position-a)*delta).sum(axis=1)/(delta*delta).sum(axis=1),0,1)
            nearest=a+t[:,None]*delta;error=np.linalg.norm(nearest-position,axis=1);s=arc[:-1]+t*np.diff(arc)
            mask=(s>=progress-.35)&(s<=progress+.45)
            ids=np.where(mask)[0];idx=ids[np.argmin(error[ids])];distance=float(error[idx]);candidate=float(s[idx])
            now=time.monotonic();mode=self.states[-1].mode.value if self.states else -1
            history.append([now-start,q[0]-initial_stamp,*q[1:],candidate,distance,mode])
            if distance>.08:
                if off_since is None:off_since=now
                if now-off_since>.6:reason=f'off line >.08m for .6s at {candidate:.3f}m';break
            else:
                off_since=None
                if candidate>progress+.003:progress=candidate;last_progress=now
            if progress>=goal:complete=True;reason='reached finish area' if self.full else 'section traversed';break
            if now-start-last_heartbeat>=10.:
                last_heartbeat=now-start
                print(f'PROGRESS {self.name}: {progress:.3f}/{arc[-1]:.3f}m ({100*progress/arc[-1]:.1f}%); lateral={distance:.4f}m; mode={mode}; elapsed={now-start:.1f}s',flush=True)
            if now-last_progress>stall_seconds:
                reason='stalled: '+(self.states[-1].detail if self.states else 'no status');break
        self.service(MissionCommand,'/mission/command',MissionCommand.Request(request_id=self.name+'-kill',command=1));self.spin(.3)
        cv2.imwrite(str(WS/f'docs/d0809/{self.name}_camera.png'),CvBridge().imgmsg_to_cv2(list(self.images.values())[-1],'bgr8'))
        print('Last image observations',[(m.image_tracking_valid,m.image_offset,m.image_heading,m.projection_valid) for _,m in list(self.lines)[-10:]],flush=True)
        values=np.array(history);events=[];last=None
        for m in self.states:
            if m.mode.value!=last:
                events.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'mode':m.mode.value,'reason':m.detail});last=m.mode.value
        summary={'run':self.name,'pass':complete,'full':self.full,'progress_m':progress,'progress_percent':100*progress/arc[-1], 'goal_m':goal,'track_length_m':float(arc[-1]),'stop_true_pose':values[-1,2:5].tolist(),'wall_sec':time.monotonic()-start,'lateral_p95_m':float(np.percentile(values[:,6],95)),'lateral_max_m':float(values[:,6].max()),'reason':reason,'events':events,'bag':str(bagdir),'candidate_reasons':sorted(set(c.reason for m in self.choices for c in m.candidates if c.selected))}
        folder=WS/'docs/d0809';folder.mkdir(parents=True,exist_ok=True)
        np.save(WS/f'log_lab/d0809/{self.name}_trajectory.npy',values)
        fig,ax=plt.subplots(figsize=(7,7));ax.plot(gt[:,0],gt[:,1],color='.75',linewidth=3,label='ordered mesh GT')
        ax.scatter(values[:,2],values[:,3],c=values[:,7],s=3,cmap='tab10',vmin=0,vmax=9);ax.plot(values[-1,2],values[-1,3],'rx',markersize=10);ax.set_aspect('equal');ax.grid();ax.set_title(f'{self.name}: {progress:.2f}m, {reason}');fig.tight_layout();fig.savefig(folder/f'{self.name}_topdown.png',dpi=110);plt.close(fig)
        fig,axes=plt.subplots(2,1,figsize=(9,5),sharex=True);axes[0].step(values[:,0],values[:,7]);axes[0].set_ylabel('mode');axes[1].plot(values[:,0],values[:,6]);axes[1].axhline(.08,color='red');axes[1].set_ylabel('lateral error m');axes[1].set_xlabel('wall s');fig.tight_layout();fig.savefig(folder/f'{self.name}_timeline.png',dpi=110);plt.close(fig)
        (folder/f'{self.name}.json').write_text(json.dumps(summary,indent=2));self.row(self.name,complete,json.dumps({k:v for k,v in summary.items() if k!='events'}));return summary
    def close(self):
        try:
            if hasattr(self,'recorder') and self.recorder.poll() is None:
                try:os.killpg(self.recorder.pid,signal.SIGINT)
                except ProcessLookupError:pass
                try:self.recorder.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    try:os.killpg(self.recorder.pid,signal.SIGKILL)
                    except ProcessLookupError:pass
                    self.recorder.wait()
        finally:
            super().close()
            import shutil
            shutil.copy2(self.log_path,WS/f'log_lab/d0809/{self.name}_launch.log')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--section',choices=['hill','corner','curve','crossing','crossing_east','left_bend','lower_loop']);parser.add_argument('--full-runs',type=int,default=2);parser.add_argument('--only-full',action='store_true');parser.add_argument('--domain',type=int,default=98);parser.add_argument('--limit',type=float,default=600);parser.add_argument('--stall-sec',type=float,default=25.);args=parser.parse_args()
    os.environ['ROS_DOMAIN_ID']=str(args.domain);signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    for path in ['docs/d0809','log_lab/d0809']: (WS/path).mkdir(parents=True,exist_ok=True)
    segments=load_centerlines(WS);reports=[];ok=True
    runs=[args.section] if args.section else ([] if args.only_full else ['hill','corner','curve','crossing'])+[f'full_{i+1}' for i in range(args.full_runs)]
    for name in runs:
        os.environ['GZ_PARTITION']=f'drive_{os.getpid()}_{name}';c=Drive(name,name.startswith('full'))
        try:c.start();reports.append(c.run(segments,args.limit,args.stall_sec))
        except Exception as exc:c.row('run completed',False,str(exc))
        finally:c.close()
        ok=ok and all(c.rows);gc.collect()
        (WS/'docs/d0809/results.json').write_text(json.dumps(reports,indent=2))
    print('D08/D09 DRIVE: '+('PASS' if ok else 'FAIL'),flush=True);return 0 if ok else 1
if __name__=='__main__':raise SystemExit(main())
