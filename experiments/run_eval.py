#!/usr/bin/env python3
"""Run one headless full-track trial of experiments/follower.py and score it against the mesh centreline.

Scoring follows tools/check_drive.py: progress along the ordered centreline, abort when the robot is
>0.08 m off the line for 0.6 s or makes no progress for --stall seconds. Ground truth comes from the
Gazebo pose stream and is used ONLY here (never by the follower).
Usage (inside the lab container): python3 experiments/run_eval.py --name t1 -- --vmax 0.22 ...
"""
import argparse, json, math, os, signal, subprocess, sys, time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
WS = HERE.parent


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--name', default='trial')
    p.add_argument('--limit', type=float, default=300)
    p.add_argument('--stall', type=float, default=20)
    p.add_argument('--off', type=float, default=0.08)
    p.add_argument('--domain', type=int, default=os.environ.get('ROS_DOMAIN_ID', '61'))
    p.add_argument('--plot', action='store_true')
    a, follower_args = p.parse_known_args()
    if follower_args and follower_args[0] == '--':
        follower_args = follower_args[1:]
    out = HERE / 'results'; out.mkdir(exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID=str(a.domain),
               GZ_PARTITION=os.environ.get('GZ_PARTITION') or f'eval_{os.getpid()}')
    os.environ.update(env)
    log = open(out / f'{a.name}.log', 'w')
    sim = subprocess.Popen(['ros2', 'launch', 'line_follower_bringup', 'sim.launch.py', 'headless:=true'],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)
    follower = None
    gt = np.load(HERE / 'gt.npy'); arc = gt[:, 2]; gt = gt[:, :2]
    truth = []

    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V
    node = Node()

    def receive(msg):
        for q in msg.pose:
            if q.name == 'turtlebot3_waffle_pi':
                truth.append((time.monotonic(), q.position.x, q.position.y, msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9))
    node.subscribe(Pose_V, '/world/line_track/dynamic_pose/info', receive)
    result = {'name': a.name, 'args': follower_args}
    try:
        t = time.monotonic()
        while not truth and time.monotonic() - t < 60:
            time.sleep(0.2)
        assert truth, 'no ground-truth pose (sim failed to start?)'
        time.sleep(4.0)  # let bridges and camera settle
        follower = subprocess.Popen([sys.executable, str(HERE / 'follower.py'), '--dump', str(out / f'{a.name}_dump.npz'), *follower_args],
                                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)
        start = time.monotonic(); progress = 0.0; last_progress = start; off_since = None
        hist = []; reason = 'time limit'; complete = False; goal = arc[-1] - 0.12; moved_at = None
        a0 = gt[:-1]; dl = np.diff(gt, axis=0); dd = (dl * dl).sum(1)
        while time.monotonic() - start < a.limit:
            time.sleep(0.05)
            _, x, y, _ = truth[-1]; pos = np.array([x, y])
            tt = np.clip(((pos - a0) * dl).sum(1) / dd, 0, 1)
            near = a0 + tt[:, None] * dl; err = np.linalg.norm(near - pos, axis=1); s = arc[:-1] + tt * np.diff(arc)
            ids = np.where((s >= progress - .35) & (s <= progress + .45))[0]
            i = ids[np.argmin(err[ids])]; dist = float(err[i]); cand = float(s[i])
            now = time.monotonic()
            if moved_at is None and cand > 0.02:
                moved_at = now
            hist.append((now - start, x, y, cand, dist))
            if dist > a.off:
                off_since = off_since or now
                if now - off_since > 0.6:
                    reason = f'off line >{a.off}m at {cand:.2f}m'; break
            else:
                off_since = None
                if cand > progress + .003:
                    progress = cand; last_progress = now
            if progress >= goal:
                complete = True; reason = 'finished'; break
            if now - last_progress > a.stall:
                reason = f'stalled at {progress:.2f}m'; break
        h = np.array(hist)
        lap = (h[-1, 0] - (moved_at - start)) if (complete and moved_at) else None
        result.update(complete=complete, progress_m=round(progress, 3), progress_pct=round(100 * progress / arc[-1], 1),
                      lap_sec=None if lap is None else round(lap, 2), wall_sec=round(h[-1, 0], 1),
                      lat_mean=round(float(h[:, 4].mean()), 4), lat_p95=round(float(np.percentile(h[:, 4], 95)), 4),
                      lat_max=round(float(h[:, 4].max()), 4), reason=reason)
        np.save(out / f'{a.name}_traj.npy', h); np.save(out / f'{a.name}_truth.npy', np.array(truth))
        if a.plot:
            import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(7, 7)); ax.plot(gt[:, 0], gt[:, 1], color='.75', lw=4)
            sc = ax.scatter(h[:, 1], h[:, 2], c=h[:, 4], s=3, cmap='viridis', vmin=0, vmax=0.05)
            fig.colorbar(sc, ax=ax, label='lateral error [m]'); ax.set_aspect('equal'); ax.grid()
            ax.set_title(f"{a.name}: {result['progress_pct']}% lap={result['lap_sec']} {reason}", fontsize=9)
            fig.tight_layout(); fig.savefig(out / f'{a.name}.png', dpi=100)
    except Exception as e:
        result.update(complete=False, reason=f'error: {e}')
    finally:
        for proc in (follower, sim):
            if proc and proc.poll() is None:
                try: os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError: pass
        for proc in (follower, sim):
            if proc:
                try: proc.wait(timeout=15); os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError: pass
                except subprocess.TimeoutExpired:
                    try: os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError: pass
    (out / f'{a.name}.json').write_text(json.dumps(result, indent=1))
    print('RESULT', json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
