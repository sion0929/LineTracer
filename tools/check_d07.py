#!/usr/bin/env python3
"""Dev-only measured hill/blind-zone validity and heading comparison."""
import argparse
import gc
import json
import math
import os
import signal
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from std_srvs.srv import Trigger
from line_follower_interfaces.msg import PathStatus
from checkpoint_a import WS,pose
from check_d06 import Moving,stamp,yaw
from d05_groundtruth import load_centerlines,distances


def angle(a):return np.arctan2(np.sin(a),np.cos(a))


class Validity(Moving):
    def __init__(self,name):
        super().__init__(name);self.samples['status']=[]
        self.node.create_subscription(PathStatus,'/path/status',lambda m:self.samples['status'].append((time.monotonic(),m)),100)

    def place(self,spawn):
        assert self.wait(lambda:self.images and self.samples['memory'] and self.data['/odom'] and self.truth,40),'startup/truth timeout'
        self.teleport(spawn)
        p=pose(self.data['/odom'][-1][1]);a=spawn[3]-p[2];c,s=math.cos(a),math.sin(a)
        self.offset=np.array([spawn[0]-c*p[0]+s*p[1],spawn[1]-s*p[0]-c*p[1],a])
        reset=self.node.create_client(Trigger,'/path/reset_memory')
        assert self.wait(reset.service_is_ready,3),'reset service missing'
        f=reset.call_async(Trigger.Request());assert self.wait(f.done,2) and f.result().success
        self.spin(1.)
        publishers=self.node.get_publishers_info_by_topic('/cmd_vel')
        self.row('dev graph command ownership',not any(n in self.node.get_node_names() for n in ('motion_controller','safety_guard')) and len(publishers)==1 and publishers[0].node_name==self.node.get_name(),
            f'publishers={[p.node_name for p in publishers]}; no motion_controller/safety_guard')

    def heading(self,segments,rate):
        center=segments[139].mean(axis=0);self.place((center[0],center[1],.01,0.))
        start=time.monotonic();self.drive_for(0,rate,.5/rate);self.drive_for(0,-rate,1./rate);self.drive_for(0,rate,.5/rate);self.drive_for(0,0,.5)
        data={k:[m for t,m in v if t>=start] for k,v in self.samples.items()}
        loc=data['localization'];truth=np.array(self.truth);truth=truth[np.argsort(truth[:,0])];truth[:,3]=np.unwrap(truth[:,3])
        errors=[]
        for m in loc:
            errors.append(abs(float(angle(yaw(m.pose.orientation)+self.offset[2]-np.interp(stamp(m),truth[:,0],truth[:,3])))))
        # Same strip-normal statistic as D-06: evaluate only the long straight
        # parallel to the initial heading; genuine crossing legs are not spread.
        rows=[];gaps=[]
        for m in data['memory']:
            for seg in m.segments:
                pts=self.odom_world([[q.x,q.y] for q in seg.points])
                if len(pts)<2:continue
                gaps.extend(np.linalg.norm(np.diff(pts,axis=0),axis=1))
                delta=pts[-1]-pts[0]
                if abs(delta[0])>.15 and abs(delta[1])<.12:
                    rows.extend(pts[(abs(pts[:,1]-center[1])<.12),1]-center[1])
        spread=float(np.std(rows)) if rows else 1.
        fresh_points=np.concatenate([self.odom_world([[q.x,q.y] for seg in m.segments for q in seg.points]) for m in data['observed']]) if data['observed'] else np.empty((0,2))
        observed_error=distances(fresh_points,segments)
        # Also map every projection-valid raw camera observation with the chosen
        # capture heading. A frozen/rejected memory must not look like zero drift.
        loc_t=np.array([stamp(m) for m in loc]);loc_yaw=np.unwrap([yaw(m.pose.orientation) for m in loc])
        raw=[];raw_frames=0
        for wall,m in self.lines:
            if wall<start or not m.projection_valid:continue
            t=stamp(m);p=self.odom_pose_at(t);a=np.interp(t,loc_t,loc_yaw)
            for seg in m.segments:
                pts=np.array([[q.x,q.y] for q in seg.points]).reshape(-1,2)
                pts=self.odom_world(pts@np.array([[math.cos(a),math.sin(a)],[-math.sin(a),math.cos(a)]])+p[:2])
                raw.extend(pts[(pts[:,0]>center[0])&(pts[:,0]<center[0]+.7)&(abs(pts[:,1]-center[1])<.12)])
            raw_frames+=1
        raw=np.asarray(raw).reshape(-1,2);bin_spreads=[]
        for x in np.arange(center[0],center[0]+.7,.05):
            values=raw[(raw[:,0]>=x)&(raw[:,0]<x+.05),1] if len(raw) else []
            if len(values)>5:bin_spreads.append(float(np.std(values)))
        raw_spread=max(bin_spreads,default=1.)
        metrics={'raw_mapped_frames':raw_frames,'raw_mapped_points':len(raw),'raw_map_spread_m':raw_spread,'mapped_observed_points' :len(fresh_points),'observed_p95_m':float(np.percentile(observed_error,95)) if len(observed_error) else None,
            'true_yaw_min_max': [float(truth[:,3].min()),float(truth[:,3].max())], 'rate':rate,'imu_yaw':os.environ.get('D07_IMU_YAW')=='true',
            'heading_error_p95_rad':float(np.percentile(errors,95)) if errors else None,
            'map_spread_m':spread,'max_memory_edge_m':float(max(gaps,default=0)),
            'localization_samples':len(loc),'truth_samples':len(truth)}
        fig,ax=plt.subplots(figsize=(6,5))
        for seg in segments:ax.plot(seg[:,0],seg[:,1],color='.8',linewidth=2)
        for m in data['memory'][::4]:
            for seg in m.segments:
                pts=self.odom_world([[q.x,q.y] for q in seg.points]);ax.plot(pts[:,0],pts[:,1],color='cyan',alpha=.25,linewidth=.7)
        ax.plot(truth[:,1],truth[:,2],'r.');ax.set_aspect('equal');ax.grid();ax.set_xlim(center[0]-.2,center[0]+1.3);ax.set_ylim(center[1]-.5,center[1]+.8)
        ax.set_title(f'gyro={metrics["imu_yaw"]}, {rate} rad/s');fig.tight_layout();fig.savefig(WS/f'docs/d07/{self.name}_topdown.png',dpi=110);plt.close(fig)
        self.row('heading measured' ,bool(errors and rows),json.dumps(metrics))
        if metrics['imu_yaw']:
            self.row('fast turn map and heading gate',raw_spread<.015 and spread<.015 and metrics['heading_error_p95_rad']<.01 and max(gaps,default=1)<.080001 and len(fresh_points)>100 and metrics['observed_p95_m']<.055 and truth[:,3].min()<-.35 and truth[:,3].max()>.4,
                f'raw spread={raw_spread:.6f}m and retained spread={spread:.6f}m <.015; true heading p95={metrics["heading_error_p95_rad"]:.6f}rad <.01; edge max={max(gaps,default=1):.6f}m')
        return metrics

    def traverse(self,segments,scenario):
        if scenario=='hill':
            start_line=segments[0,1];spawn=(start_line[0],start_line[1],.01,0.)
            distance=segments[10,:,0].mean()-spawn[0];velocity=.10
        else:
            end=segments[-1].mean(axis=0);spawn=(end[0]+.70,end[1],.01,np.pi)
            distance=.82;velocity=.06
        self.place(spawn);start=time.monotonic()
        self.drive_for(velocity,0,distance/velocity);self.drive_for(0,0,2.)
        truth=np.asarray(self.truth);truth=truth[np.argsort(truth[:,0])];truth[:,3]=np.unwrap(truth[:,3])
        statuses=[m for t,m in self.samples['status'] if t>=start]
        paths=[m for t,m in self.samples['local'] if t>=start]
        memory=[m for t,m in self.samples['memory'] if t>=start]
        observations=[m for t,m in self.lines if t>=start]
        timeline=[];last=None
        for m in statuses:
            if m.mode.value!=last:
                timeline.append({'t':stamp(m),'mode':m.mode.value,'remaining':m.verified_remaining_distance,'uncertainty':m.uncertainty,'detail':m.detail});last=m.mode.value
        errors=[];mem_errors=[];regions={k:[] for k in ('flat','entry_uphill','top','downhill_exit')}
        observed_points=[];memory_points=[];loss=[]
        for m in observations:
            t=stamp(m);x=np.interp(t,truth[:,0],truth[:,1]);y=np.interp(t,truth[:,0],truth[:,2]);a=np.interp(t,truth[:,0],truth[:,3])
            pts=np.array([[q.x,q.y] for seg in m.segments for q in seg.points]).reshape(-1,2)
            world=pts@np.array([[math.cos(a),math.sin(a)],[-math.sin(a),math.cos(a)]])+[x,y]
            e=distances(world,segments)
            if m.projection_valid and len(e):errors.extend(e);observed_points.extend(world)
            imu=min((q for _,q in self.data['/imu']),key=lambda q:abs(stamp(q)-t));q=imu.orientation
            pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
            region='flat' if x<.66 else 'entry_uphill' if x<1.10 else 'top' if x<1.54 else 'downhill_exit'
            regions[region].append({'t':t,'x':x,'pitch':pitch,'valid':m.projection_valid,
                'nearest':m.observed_range_min if m.projection_valid else None,'farthest':m.observed_range_max if m.projection_valid else None,
                'max_error':float(max(e)) if len(e) and m.projection_valid else None})
            loss.append((t,not m.projection_valid or not m.line_found))
        for m in memory:
            pts=self.odom_world([[p.x,p.y] for seg in m.segments for p in seg.points]);memory_points.extend(pts);mem_errors.extend(distances(pts,segments))
        modes=[e['mode'] for e in timeline];unknown=sum(q.source==2 for m in paths for q in m.points)
        quant=lambda a: {'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'max':float(max(a))} if len(a) else None
        continuous=0.;begin=None
        for t,missing in loss:
            if missing and begin is None:begin=t
            if not missing and begin is not None:continuous=max(continuous,t-begin);begin=None
        if begin is not None and loss:continuous=max(continuous,loss[-1][0]-begin)
        summary={'scenario':scenario,'confirmed_error_m':quant(errors),'memory_error_m':quant(mem_errors),
            'mode_timeline':timeline,'unknown_points':unknown,'max_continuous_loss_sec':continuous,
            'samples':{'line':len(observations),'memory':len(memory),'path':len(paths),'status':len(statuses)},
            'observation_regions':{k:{'frames':len(v),'valid_frames':sum(r['valid'] for r in v),
                'nearest':min((r['nearest'] for r in v if r['nearest']),default=None),
                'farthest':max((r['farthest'] for r in v if r['farthest']),default=None),
                'max_error':max((r['max_error'] for r in v if r['max_error'] is not None),default=None)} for k,v in regions.items()}}
        self.row('confirmed and memory ground truth',bool(errors and mem_errors) and np.median(errors)<.025 and np.percentile(errors,95)<.045 and max(errors)<.08 and np.percentile(mem_errors,95)<.055 and max(mem_errors)<.10,json.dumps({k:summary[k] for k in ('scenario','confirmed_error_m','memory_error_m')}))
        expired=sum(q.uncertainty>=.06 or q.valid_until.sec+q.valid_until.nanosec*1e-9<=stamp(m) for m in paths for q in m.points)
        self.row('no UNKNOWN or uncertain/expired confirmed points',bool(paths) and unknown==0 and expired==0,f'paths={len(paths)} UNKNOWN={unknown} uncertain/expired={expired}')
        if scenario=='hill':
            exit_x=segments[8,:,0].max()+.16
            recovered=[m for m in statuses if m.mode.value==0 and np.interp(stamp(m),truth[:,0],truth[:,1])>exit_x]
            self.row('loss and checked recovery',1 in modes and 2 in modes and len(recovered)>10,
                f'modes={modes}; VISION samples beyond exit+0.16m={len(recovered)}; final mode={modes[-1] if modes else None}')
            slope=[r for records in regions.values() for r in records if abs(r['pitch'])>.035]
            self.row('capture pitch validity gate',bool(slope) and not any(r['valid'] for r in slope),f'slope frames={len(slope)}; confirmed={sum(r["valid"] for r in slope)}')
        else:
            slows=[m for m in statuses if m.mode.value==3];stops=[m for m in statuses if m.mode.value==5]
            margin=stops[0].verified_remaining_distance if stops else None
            summary['stop_margin_m']=margin
            positive=[m.verified_remaining_distance for m in statuses if m.verified_remaining_distance>0]
            self.row('verified distance exhaustion',bool(slows and stops and positive) and max(positive)-min(positive)>.3 and margin is not None and margin>=0,
                f'SLOWDOWN={len(slows)} STOPPING={len(stops)} first stop remaining={margin}; remaining range={min(positive,default=0):.4f}..{max(positive,default=0):.4f}')
        folder=WS/'docs/d07';folder.mkdir(parents=True,exist_ok=True)
        trajectory=truth[(truth[:,0]>=stamp(statuses[0]))] if statuses else truth
        fig,ax=plt.subplots(figsize=(8,4))
        for seg in segments:ax.plot(seg[:,0],seg[:,1],color='.75',linewidth=2)
        for pts,label,color in [(observed_points,'confirmed observed','green'),(memory_points,'memory','cyan')]:
            pts=np.asarray(pts).reshape(-1,2)
            if len(pts):ax.scatter(pts[:,0],pts[:,1],s=1,color=color,label=label)
        ax.plot(trajectory[:,1],trajectory[:,2],'r-',label='true robot trajectory');ax.set_aspect('equal');ax.grid();ax.legend();ax.set_title(scenario)
        ax.set_xlim(trajectory[:,1].min()-.4,trajectory[:,1].max()+.8);ax.set_ylim(spawn[1]-.4,spawn[1]+.4)
        fig.tight_layout();fig.savefig(folder/f'{scenario}_topdown.png',dpi=110);plt.close(fig)
        fig,axes=plt.subplots(3,1,figsize=(9,6),sharex=True)
        t0=stamp(statuses[0]) if statuses else 0
        ts=[stamp(m)-t0 for m in statuses]
        axes[0].step(ts,[m.mode.value for m in statuses],where='post');axes[0].set_ylabel('mode (0..5)')
        axes[1].plot(ts,[m.verified_remaining_distance for m in statuses],label='verified [m]');axes[1].plot(ts,[m.uncertainty for m in statuses],label='uncertainty [m]');axes[1].legend()
        axes[2].plot([stamp(m)-t0 for m in paths],[m.speed_limit for m in paths]);axes[2].set_ylabel('speed limit [m/s]');axes[2].set_xlabel('simulation elapsed [s]')
        fig.tight_layout();fig.savefig(folder/f'{scenario}_timeline.png',dpi=110);plt.close(fig)
        (WS/f'log_lab/d07/{scenario}_frames.json').write_text(json.dumps(regions,indent=2))
        self.row('mode timeline',True,json.dumps(timeline))
        return summary


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--heading-only',action='store_true');parser.add_argument('--scenario',choices=['hill','blind']);parser.add_argument('--imu-yaw',choices=['false','true','both'],default='both');args=parser.parse_args()
    os.environ['ROS_DOMAIN_ID']='97'
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    output=WS/'docs/d07';output.mkdir(parents=True,exist_ok=True)
    logs=WS/'log_lab/d07';logs.mkdir(parents=True,exist_ok=True)
    reports=[];ok=True;segments=load_centerlines(WS)
    for source in (() if args.scenario else (('false','true') if args.imu_yaw=='both' else (args.imu_yaw,))):
        for rate in (.25,.5):
            os.environ['D07_IMU_YAW']=source;os.environ['GZ_PARTITION']=f'd07_{os.getpid()}_{source}_{rate}'
            c=Validity(f'heading_{source}_{rate}')
            try:c.start();reports.append(c.heading(segments,rate))
            except Exception as exc:c.row('scenario completed',False,str(exc))
            finally:c.close()
            (output/'measurements_in_progress.json').write_text(json.dumps(reports,indent=2))
            ok=ok and all(c.rows);gc.collect()
    if not args.heading_only:
        os.environ['D07_IMU_YAW']='true'
        for scenario in ((args.scenario,) if args.scenario else ('hill','blind')):
            os.environ['GZ_PARTITION']=f'd07_{os.getpid()}_{scenario}'
            c=Validity(scenario)
            try:c.start();reports.append(c.traverse(segments,scenario))
            except Exception as exc:c.row('scenario completed',False,str(exc))
            finally:c.close()
            (output/'measurements_in_progress.json').write_text(json.dumps(reports,indent=2))
            ok=ok and all(c.rows);gc.collect()
    (output/('heading.json' if args.heading_only else 'measurements.json')).write_text(json.dumps(reports,indent=2))
    print('D-07'+(' heading measurements' if args.heading_only else '')+': '+('PASS' if ok else 'FAIL'),flush=True)
    return 0 if ok else 1


if __name__=='__main__':raise SystemExit(main())
