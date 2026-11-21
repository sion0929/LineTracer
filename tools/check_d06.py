#!/usr/bin/env python3
"""Dev-only commanded moving tests of capture-pose mapping and local memory against the mesh."""
import gc
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from line_follower_interfaces.msg import ObservedGraph,LocalPath,LocalizationEstimate,GcsSnapshot
from checkpoint_a import WS,pose,speed,zero
from check_d05 import Capture
from d05_groundtruth import load_centerlines,distances


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9


class Moving(Capture):
    def __init__(self,name):
        super().__init__();self.name=name;self.samples={t:[] for t in ('observed','memory','local','localization','snapshot')}
        self.node.destroy_subscription(self.subs[3])
        self.subs[3]=self.node.create_subscription(Odometry,'/odom',lambda m:self.record('/odom',m),1000)
        self.last_sample={};self.pub=self.node.create_publisher(Twist,'/cmd_vel',10)
        for topic,typ,key in [('/path/observed_graph',ObservedGraph,'observed'),('/path/memory',ObservedGraph,'memory'),
                              ('/path/local',LocalPath,'local'),('/localization/estimate',LocalizationEstimate,'localization'),
                              ('/mission/gcs_snapshot',GcsSnapshot,'snapshot')]:
            self.node.create_subscription(typ,topic,lambda m,k=key:self.collect(k,m),30)

    def collect(self,key,msg):
        now=time.monotonic()
        if now-self.last_sample.get(key,0)<.15 and key not in ('localization','snapshot'):return
        self.last_sample[key]=now;self.samples[key].append((now,msg))

    def start(self):
        self.launch=subprocess.Popen(['ros2','launch',str(WS/'tools/d06_capture.launch.py')],
            stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        self.start_truth()

    def drive_for(self,v,w,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            self.command(v,w);self.spin(.04)

    def odom_pose_at(self,t):
        records=[m for _,m in self.data['/odom']]
        return pose(min(records,key=lambda m:abs(stamp(m)-t)))

    def odom_world(self,points):
        points=np.asarray(points).reshape(-1,2);c,s=math.cos(self.offset[2]),math.sin(self.offset[2])
        return points@np.array([[c,s],[-s,c]])+self.offset[:2]

    def estimate_pose_at(self,t):
        records=[m for _,m in self.samples['localization'] if m.healthy]
        m=min(records,key=lambda m:abs(stamp(m)-t))
        return np.array([m.pose.position.x,m.pose.position.y,yaw(m.pose.orientation)])

    def local_world(self,msg,source=None):
        p=self.estimate_pose_at(stamp(msg));c,s=math.cos(p[2]),math.sin(p[2])
        pts=np.array([[q.x,q.y] for q in msg.points if source is None or q.source==source]).reshape(-1,2)
        return self.odom_world(pts@np.array([[c,s],[-s,c]])+p[:2])

    def run(self,segments,scenario):
        # All placements derive from verified mesh centerlines, never runtime waypoints.
        if scenario=='straight':
            center=segments[139].mean(axis=0);spawn=(center[0]-.30,center[1],.01,0.)
        elif scenario=='corner':
            corner=segments[99,1,:2];spawn=(corner[0],corner[1]-.60,.01,np.pi/2)
        else:
            center=segments[139].mean(axis=0);spawn=(center[0],center[1],.01,0.)
        assert self.wait(lambda:self.images and self.samples['memory'] and self.data['/odom'],35),'startup timeout'
        self.teleport(spawn)
        p=pose(self.data['/odom'][-1][1]);angle=spawn[3]-p[2];c,s=math.cos(angle),math.sin(angle)
        self.offset=np.array([spawn[0]-c*p[0]+s*p[1],spawn[1]-s*p[0]-c*p[1],angle])
        # A teleport changes the physical odom/world relation without wheel travel.
        # Clear pre-placement observations through the same fault-reset service.
        reset=self.node.create_client(Trigger,'/path/reset_memory')
        assert self.wait(reset.service_is_ready,3.),'memory reset service missing'
        future=reset.call_async(Trigger.Request())
        assert self.wait(future.done,2.) and future.result().success,'memory reset rejected'
        self.spin(1.)
        start=time.monotonic();initial=pose(self.data['/odom'][-1][1])
        if scenario=='straight':self.drive_for(.07,0,10.)
        elif scenario=='corner':self.drive_for(.06,0,6.5)
        else:
            self.drive_for(0,.1,5.);self.drive_for(0,-.1,10.);self.drive_for(0,.1,5.)
        self.drive_for(0,0,.6)
        end=time.monotonic();final=pose(self.data['/odom'][-1][1])
        publishers=self.node.get_publishers_info_by_topic('/cmd_vel')
        self.row('dev graph command ownership',not any(n in self.node.get_node_names() for n in ('motion_controller','safety_guard')) and
                 len(publishers)==1 and publishers[0].node_name==self.node.get_name(),
                 f'no controller/guard; publishers={[p.node_name for p in publishers]}; scenario={scenario}')
        observed=[m for t,m in self.samples['observed'] if t>=start]
        memory=[m for t,m in self.samples['memory'] if t>=start]
        local=[m for t,m in self.samples['local'] if t>=start and m.points]
        obs_points=np.concatenate([self.odom_world([[q.x,q.y] for s in m.segments for q in s.points]) for m in observed]) if observed else np.empty((0,2))
        mem_points=np.concatenate([self.odom_world([[q.x,q.y] for s in m.segments for q in s.points]) for m in memory]) if memory else np.empty((0,2))
        oe=distances(obs_points,segments);me=distances(mem_points,segments)
        quant=lambda a:[float(np.median(a)),float(np.percentile(a,95)),float(np.max(a))] if len(a) else [None]*3
        counts=[sum(len(s.points) for s in m.segments) for m in memory]
        unknown=sum(q.source==2 for m in local for q in m.points)
        healthy=[m for t,m in self.samples['localization'] if t>=start and m.healthy]
        loc_errors=[]
        for m in healthy:
            p=list(self.odom_pose_at(stamp(m)))
            truth=np.asarray(self.truth);truth=truth[np.argsort(truth[:,0])]
            p[2]=np.interp(stamp(m),truth[:,0],np.unwrap(truth[:,3]))-self.offset[2]
            loc_errors.append([math.hypot(m.pose.position.x-p[0],m.pose.position.y-p[1]),abs(math.remainder(yaw(m.pose.orientation)-p[2],2*math.pi))])
        snap_errors=[]
        for t,m in self.samples['snapshot']:
            if t<start or not m.pose_valid:continue
            p=self.estimate_pose_at(stamp(m));snap_errors.append([math.hypot(m.robot_pose.position.x-p[0],m.robot_pose.position.y-p[1]),abs(math.remainder(yaw(m.robot_pose.orientation)-p[2],2*math.pi))])
        # Normal-distance spread per fixed world bin, rather than tracking resampled point indices.
        spread=[]
        if scenario in ('straight','rotate') and len(mem_points):
            target_y=spawn[1]
            aligned=[]
            for msg in memory:
                for seg in msg.segments:
                    if len(seg.points)<2:continue
                    pts=self.odom_world([[p.x,p.y] for p in seg.points])
                    delta=pts[-1]-pts[0]
                    if abs(delta[0])>5*abs(delta[1]):aligned.extend(pts)
            aligned=np.asarray(aligned).reshape(-1,2)
            roi=aligned[np.abs(aligned[:,1]-target_y)<.05]
            for x in np.arange(spawn[0]-.05,spawn[0]+1.2,.05):
                values=roi[(roi[:,0]>=x)&(roi[:,0]<x+.05),1]
                if len(values)>5:spread.append(float(np.std(values)))
        yaw_errors=[]
        for ts,msg in self.data['/imu']:
            if ts<start:continue
            p=self.odom_pose_at(stamp(msg))
            yaw_errors.append(math.remainder(yaw(msg.orientation)-p[2]-self.offset[2],2*math.pi))
        diagnostic=[]
        for ts,msg in self.samples['memory']:
            if ts<start:continue
            for seg in msg.segments:
                pts=self.odom_world([[p.x,p.y] for p in seg.points])
                diagnostic.extend([[ts-start,float(p[0]),float(p[1]),seg.id] for p in pts])
        samples_path=WS/f'log_lab/d06/{scenario}_memory_samples.npy'
        samples_path.parent.mkdir(parents=True,exist_ok=True)
        np.save(samples_path,np.asarray(diagnostic))
        summary={'scenario':scenario,'imu_odom_yaw_max_difference':max(map(abs,yaw_errors),default=0.),'observed_error_m':quant(oe),'memory_error_m':quant(me),
                 'memory_peak_points':max(counts,default=0),'memory_end_points':counts[-1] if counts else 0,
                 'memory_peak_segments':max((len(m.segments) for m in memory),default=0),
                 'normal_spread_max_m':max(spread,default=0.),'unknown_points':unknown,
                 'localization_max_error':np.max(loc_errors,axis=0).tolist() if loc_errors else None,
                 'snapshot_max_error':np.max(snap_errors,axis=0).tolist() if snap_errors else None,
                 'displacement_m':math.hypot(final[0]-initial[0],final[1]-initial[1]),
                 'observed_samples':len(observed),'memory_samples':len(memory),'path_samples':len(local),
                 'memory_source_points':sum(q.source==1 for m in local for q in m.points),
                 'spawn_world':list(map(float,spawn)),'odom_to_world':self.offset.tolist()}
        geometry=len(oe)>100 and len(me)>100 and np.median(oe)<.03 and np.percentile(oe,95)<.055 and np.max(oe)<.10 and np.percentile(me,95)<.055 and np.max(me)<.10
        self.row('observed/memory ground truth',geometry,json.dumps(summary,sort_keys=True))
        self.row('bounded deduplicated memory',bool(counts) and max(counts)<=1200 and summary['memory_peak_segments']<=32 and counts[-1]<600,
                 f'peak/end points={max(counts,default=0)}/{counts[-1] if counts else 0}; segments={summary["memory_peak_segments"]}')
        self.row('frame and source contract',bool(local) and not unknown and all(m.header.frame_id=='odom' for m in observed+memory) and
                 all(m.header.frame_id=='base_footprint' for m in local),f'unknown={unknown}; observed/memory=odom; local=base_footprint')
        self.row('odom XY + true heading and snapshot',bool(loc_errors) and bool(snap_errors) and max(e[0] for e in loc_errors)<.002 and max(e[1] for e in loc_errors)<.005 and
                 max(e[0] for e in snap_errors)<.015 and max(e[1] for e in snap_errors)<.03,
                 f'loc max={summary["localization_max_error"]}; snapshot max={summary["snapshot_max_error"]}')
        if scenario=='straight':
            near=[m for ts,m in self.samples['local'] if ts>end-.5]
            blind=[q.x for m in near for q in m.points if q.source==1 and 0<=q.x<.26 and abs(q.y)<.05]
            summary['blind_memory_range_m']=[min(blind),max(blind)] if blind else None
            self.row('straight blind zone uses memory',bool(blind) and min(blind)<.12 and max(blind)>.18,
                     f'MEMORY_UPDATED near range={summary["blind_memory_range_m"]}')
            self.row('forward distance and stable map',.6<summary['displacement_m']<.85 and max(spread,default=1)<.015,
                     f'displacement={summary["displacement_m"]:.3f}m; normal spread={max(spread,default=1):.5f}m')
        elif scenario=='corner':
            near=[m for t,m in self.samples['local'] if t>end-.5 and m.points]
            pts=np.concatenate([self.local_world(m,1) for m in near]) if near else np.empty((0,2))
            corner=segments[99,1,:2];d=np.linalg.norm(pts-corner,axis=1) if len(pts) else np.array([])
            vertex=float(np.min(d)) if len(d) else None
            outgoing=bool(len(pts) and np.any((pts[:,0]>corner[0]+.07)&(np.abs(pts[:,1]-corner[1])<.03)))
            robot=self.odom_world([[final[0],final[1]]])[0];remaining=float(corner[1]-robot[1])
            angle=None
            if near:
                chain=self.local_world(near[-1],1)
                if len(chain)>6:
                    k=int(np.argmin(np.linalg.norm(chain-corner,axis=1)))
                    before=next((i for i in range(k-1,-1,-1) if np.linalg.norm(chain[i]-chain[k])>=.06),None)
                    after=next((i for i in range(k+1,len(chain)) if np.linalg.norm(chain[i]-chain[k])>=.06),None)
                    if before is not None and after is not None:
                        a=chain[k]-chain[before];b=chain[after]-chain[k]
                        denom=np.linalg.norm(a)*np.linalg.norm(b)
                        if denom>1e-8:angle=float(np.degrees(np.arccos(np.clip(a@b/denom,-1,1))))
            summary.update(corner_memory_error_m=vertex,corner_forward_m=remaining,outgoing_memory=outgoing,corner_angle_deg=angle)
            self.row('blind corner remains sharp memory',vertex is not None and vertex<.015 and outgoing and .12<remaining<.30 and angle is not None and 75<angle<105,
                     f'vertex error={vertex}; outgoing={outgoing}; angle={angle}; corner ahead={remaining:.3f}m < nearest view~0.316m')
        else:
            odoms=[pose(m) for t,m in self.data['/odom'] if t>=start]
            excursion=max(abs(math.remainder(p[2]-initial[2],2*math.pi)) for p in odoms)
            summary['rotation_peak_rad']=excursion
            self.row('rotation does not smear map',.43<excursion<.65 and summary['displacement_m']<.02 and max(spread,default=1)<.015,
                     f'peak yaw={excursion:.3f}rad; translation={summary["displacement_m"]:.4f}m; spread={max(spread,default=1):.5f}m')
        self.row('stop leaves zero',zero(self.data['/cmd_vel'][-1][1]) and speed(self.data['/odom'][-1][1])[0]<.008 and speed(self.data['/odom'][-1][1])[1]<.02,
                 f'last_zero={zero(self.data["/cmd_vel"][-1][1])}; speed={speed(self.data["/odom"][-1][1])}')
        self.plot(segments,observed,memory,local,start)
        return summary

    def plot(self,segments,observed,memory,local,start):
        fig,ax=plt.subplots(figsize=(7,7))
        for line in segments:ax.plot(line[:,0],line[:,1],color='.8',lw=2)
        for messages,color,label in [(observed,'green','direct observed'),(memory,'cyan','retained memory')]:
            for m in messages[::max(1,len(messages)//12)]:
                for s in m.segments:
                    pts=self.odom_world([[p.x,p.y] for p in s.points]);ax.plot(pts[:,0],pts[:,1],color=color,alpha=.4,lw=1)
            ax.plot([],[],color=color,label=label)
        for m in local[::max(1,len(local)//12)]:
            pts=self.local_world(m,1)
            if len(pts):ax.scatter(pts[:,0],pts[:,1],color='blue',s=3)
        ax.plot([],[],'b.',label='memory in local path')
        trajectory=self.odom_world([[pose(m)[0],pose(m)[1]] for t,m in self.data['/odom'] if t>=start])
        ax.plot(trajectory[:,0],trajectory[:,1],'r-',label='robot trajectory')
        odoms=[pose(m) for ts,m in self.data['/odom'] if ts>=start]
        for k in range(0,len(odoms),max(1,len(odoms)//12)):
            heading=odoms[k][2]+self.offset[2]
            ax.arrow(trajectory[k,0],trajectory[k,1],.09*math.cos(heading),.09*math.sin(heading),
                     width=.002,head_width=.015,color='r',alpha=.5,length_includes_head=True)
        if self.name=='corner':
            landmark=segments[99,1,:2];ax.plot(*landmark,'kx',label='mesh corner')
        ax.set_aspect('equal');ax.set_xlim(trajectory[:,0].min()-.4,trajectory[:,0].max()+1.25);ax.set_ylim(trajectory[:,1].min()-.45,trajectory[:,1].max()+1.25)
        ax.set_xlabel('world x [m]');ax.set_ylabel('world y [m]');ax.set_title(self.name);ax.grid();ax.legend(fontsize=8);fig.tight_layout()
        plot_path=WS/f'docs/d06/{self.name}_topdown.png'
        plot_path.parent.mkdir(parents=True,exist_ok=True)
        fig.savefig(plot_path,dpi=110);plt.close(fig)


def main():
    os.environ['ROS_DOMAIN_ID']='95'
    def interrupted(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupted)
    output=WS/'docs/d06';output.mkdir(parents=True,exist_ok=True)
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--scenario',choices=['straight','corner','rotate']);args=parser.parse_args()
    segments=load_centerlines(WS);reports=[];ok=True
    print('D06 gates: observed median/p95/max <.030/.055/.100m; memory p95/max <.055/.100m; spread<.015m; caps1200points/32segments; localization .002m/.005rad; snapshot .015m/.03rad',flush=True)
    for scenario in ((args.scenario,) if args.scenario else ('straight','corner','rotate')):
        os.environ['GZ_PARTITION']=f'd06_{os.getpid()}_{scenario}'
        c=Moving(scenario)
        try:c.start();reports.append(c.run(segments,scenario))
        except (Exception,KeyboardInterrupt) as exc:c.row('scenario completed',False,str(exc))
        finally:c.close()
        ok=ok and all(c.rows)
        gc.collect()
    (output/'measurements.json').write_text(json.dumps(reports,indent=2))
    print('D-06: '+('PASS' if ok else 'FAIL'),flush=True)
    return 0 if ok else 1


if __name__=='__main__':raise SystemExit(main())
