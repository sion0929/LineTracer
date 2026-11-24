#!/usr/bin/env python3
"""Tune the corner behaviour of the user pure-pursuit controller in the 2D simulator (dev tool).

45.7 % of the Gazebo baseline failures (hills60 + official_like50) end at a corner. The user
controller approaches a corner at the curvature speed and, when the lookahead target is more
than 60 deg off, pivots with w = 2.5 * alpha (up to 3 rad/s) at v_min. This script varies those
rules without touching the package, in parallel worker processes:

  a_lat      lateral acceleration for the curvature speed limit
  v_corner   speed cap within `brake_d` of a sharp heading change (>= 45 deg) on the known path
  pivot_deg  alpha above which the robot turns on the spot
  pivot_gain / w_max  how hard it turns
  ld_min / ld_gain    pure-pursuit lookahead

  python experiments/eval/corner_tune_2d.py --rl-root .../b-dlseg --episodes 40 --workers 8 --out out.json
"""
import argparse
import itertools
import json
import math
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

RL_ROOT = None


def controller(p):
    def ctrl(path, end, previous, blind, DT):
        if len(path) < 2:
            return np.array([0.06, 0.0] if blind < 0.35 else [0.0, 0.8])
        v_max, v_min = p["v_max"], 0.06
        speed_limit = v_max
        length, anchor, old_heading = math.hypot(*path[0]), path[0], None
        sharp_at = None
        for point in path[1:]:
            dx, dy = point - anchor
            seg = math.hypot(dx, dy)
            if seg < 0.05:
                continue
            heading = math.atan2(dy, dx)
            if old_heading is not None:
                turn = abs(math.remainder(heading - old_heading, math.tau))
                kappa = turn / seg
                v_c = max(v_min, min(v_max, math.sqrt(p["a_lat"] / max(kappa, 1e-3))))
                speed_limit = min(speed_limit, math.sqrt(v_c ** 2 + 0.6 * max(0.0, length - 0.10)))
                if sharp_at is None and turn > math.radians(45) * (seg / 0.05):
                    sharp_at = length
            old_heading, anchor = heading, point
            length += seg
        if sharp_at is not None and p["v_corner"] < v_max:
            d = max(0.0, sharp_at - p["brake_d"])
            speed_limit = min(speed_limit, math.sqrt(p["v_corner"] ** 2 + 2 * 0.3 * d))
        if end == "open":
            speed_limit = min(speed_limit, math.sqrt(v_min ** 2 + 0.6 * max(0.0, length - 0.10)))
        v = max(previous[0] - DT, min(speed_limit, previous[0] + 0.60 * DT))
        ld = p["ld_min"] + p["ld_gain"] * v
        far = np.flatnonzero(np.einsum("ij,ij->i", path, path) >= ld * ld)
        x, y = path[far[0]] if len(far) else path[-1]
        alpha = math.atan2(y, x)
        if abs(alpha) > math.radians(p["pivot_deg"]):
            return np.array([min(v, v_min), float(np.clip(p["pivot_gain"] * alpha, -p["w_max"], p["w_max"]))])
        return np.array([v, float(np.clip(2.0 * v * y / max(1e-4, x * x + y * y), -p["w_max"], p["w_max"]))])
    return ctrl


def run_one(args):
    name, p, seed, diff, maps = args
    sys.path.insert(0, RL_ROOT)
    from experiments.rl import simulator as S
    from experiments.rl.policy import DT
    sim = S.Simulator(Path(maps) / f"w{seed % 8}")
    sim.reset(seed, 10_000 + seed, training=True, difficulty=diff)
    ctrl = controller(p)
    reason = None
    while reason is None:
        _, _, reason = sim.step(ctrl(sim.local_path(), sim.end, sim.previous, sim.blind, DT))
    r = sim.result(reason)
    return name, diff, r["complete"], r["progress_pct"], r["sim_sec"], r["wheel_line_events"]


def init(root):
    global RL_ROOT
    RL_ROOT = root


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rl-root", required=True)
    ap.add_argument("--episodes", type=int, default=40)
    ap.add_argument("--seed0", type=int, default=3000)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    base = dict(v_max=0.25, a_lat=0.25, v_corner=9.0, brake_d=0.10, pivot_deg=60.0, pivot_gain=2.5,
                w_max=3.0, ld_min=0.10, ld_gain=0.30)
    grid = {"base(user)": base}
    for a_lat, vc, piv, wmax in itertools.product((0.25, 0.12), (9.0, 0.10, 0.06), (60.0, 40.0), (3.0, 1.5)):
        name = f"alat{a_lat}_vc{vc}_piv{piv:.0f}_w{wmax}"
        grid[name] = dict(base, a_lat=a_lat, v_corner=vc, pivot_deg=piv, w_max=wmax)
    for ldm in (0.06, 0.14):
        grid[f"ld{ldm}"] = dict(base, ld_min=ldm)
    diffs = ("easy", "normal", "hard")
    maps = str(Path(a.out).parent / "maps2d_tune")
    jobs = [(n, p, a.seed0 + k, diffs[k % 3], maps) for n, p in grid.items() for k in range(a.episodes)]
    t0 = time.time()
    with Pool(a.workers, initializer=init, initargs=(a.rl_root,)) as pool:
        out = pool.map(run_one, jobs, chunksize=4)
    res = {}
    for n in grid:
        rows = [o for o in out if o[0] == n]
        done = [o for o in rows if o[2]]
        res[n] = dict(params=grid[n], complete=len(done), episodes=len(rows),
                      mean_progress=round(float(np.mean([o[3] for o in rows])), 1),
                      lap=round(float(np.mean([o[4] for o in done])), 1) if done else None,
                      by_diff={d: sum(o[2] for o in rows if o[1] == d) for d in diffs})
    for n, r in sorted(res.items(), key=lambda kv: (-kv[1]["complete"], -kv[1]["mean_progress"])):
        print(f"{n:32} {r['complete']:3}/{r['episodes']}  mean {r['mean_progress']:5.1f}%  lap {r['lap']}  {r['by_diff']}")
    print(f"wall {time.time() - t0:.0f} s")
    Path(a.out).write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
