#!/usr/bin/env python3
"""Compare path-tracking controllers in the fast 2D simulator (dev tool).

The simulator (experiments/rl/simulator.py, by the DL track) models wheel slip, command
delay, odometry error, the 0.31 m camera blind zone and the camera-built line map, and
reproduced Gazebo lap times of the user package (67-70 s vs 63-72 s).

Controllers (all consume the same map path in the robot frame):
  user_pp          user package pure pursuit WITH the official-track speed_profile
  user_pp_flat     same, speed_profile removed (top speed v_max everywhere)
  rpp_*            Nav2 Regulated Pure Pursuit rules (adaptive lookahead, curvature-regulated
                   speed, approach slow-down on an open path end, rotate-in-place for large
                   heading error), ported to Python with small-robot parameters

  python experiments/eval/controllers_2d.py --rl-root .../b-dlseg --episodes 60 --out out.json
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np


def rpp_factory(v_des, lt=1.2, l_min=0.10, l_max=0.45, r_min=0.25, rot_deg=60.0, w_max=3.0,
                approach=0.25, v_min=0.06):
    """Nav2 RPP core: lookahead = clamp(v*lt), curvature regulation below r_min,
    approach velocity scaling as the known path runs out, rotate-to-heading."""
    def ctrl(path, end, previous):
        if len(path) < 2:
            return None
        v_prev = float(previous[0])
        L = min(max(abs(v_prev) * lt, l_min), l_max)
        d2 = np.einsum("ij,ij->i", path, path)
        far = np.flatnonzero(d2 >= L * L)
        x, y = path[far[0]] if len(far) else path[-1]
        dist2 = max(1e-4, x * x + y * y)
        alpha = math.atan2(y, x)
        if abs(alpha) > math.radians(rot_deg):
            return np.array([0.0, float(np.clip(2.5 * alpha, -w_max, w_max))])
        kappa = 2.0 * y / dist2
        v = v_des
        r = 1.0 / max(abs(kappa), 1e-6)
        if r < r_min:
            v *= r / r_min                                   # curvature regulation
        # also look further along the path for the tightest curvature (brake early)
        seg = np.diff(path, axis=0)
        h = np.arctan2(seg[:, 1], seg[:, 0])
        ds = np.linalg.norm(seg, axis=1)
        s = np.cumsum(ds)
        dh = np.abs(np.remainder(np.diff(h) + math.pi, 2 * math.pi) - math.pi)
        k_ahead = dh / np.maximum(ds[1:], 1e-3)
        for kk, sk in zip(k_ahead, s[1:]):
            if kk > 1e-3:
                rk = 1.0 / kk
                if rk < r_min:
                    v_k = max(v_min, v_des * rk / r_min)
                    v = min(v, math.sqrt(v_k ** 2 + 2 * 0.6 * max(0.0, sk - 0.10)))
        remaining = float(s[-1]) if len(s) else 0.0
        if end == "open" and remaining < approach:
            v = min(v, max(v_min, v_des * remaining / approach))   # approach scaling
        v = max(v, v_min)
        w = float(np.clip(v * kappa, -w_max, w_max))
        return np.array([v, w])
    return ctrl


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rl-root", required=True, help="worktree that contains experiments/rl/simulator.py")
    ap.add_argument("--episodes", type=int, default=60)
    ap.add_argument("--seed0", type=int, default=2000)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    sys.path.insert(0, a.rl_root)
    from experiments.rl import simulator as S
    from experiments.rl.policy import limit, DT  # noqa: F401

    maps = Path(a.out).parent / "maps2d"
    sim = S.Simulator(maps)

    def user_pp(profile):
        def ctrl(path, end, previous, travel, blind):
            saved = S.PROFILE
            if not profile:
                S.PROFILE = ()
            try:
                return S.pure_pursuit(path, end, travel, previous, blind)
            finally:
                S.PROFILE = saved
        return ctrl

    cands = {"user_pp": user_pp(True), "user_pp_flat": user_pp(False)}
    for v in (0.25, 0.35, 0.45):
        for rmin in (0.20, 0.35):
            cands[f"rpp_v{v}_r{rmin}"] = rpp_factory(v, r_min=rmin)

    results = {}
    diffs = ("easy", "normal", "hard")
    for name, ctrl in cands.items():
        rows = []
        t0 = time.time()
        for k in range(a.episodes):
            seed = a.seed0 + k
            sim.reset(seed, 10_000 + seed, training=True, difficulty=diffs[k % 3])
            reason = None
            while reason is None:
                path = sim.local_path()
                if name.startswith("user_pp"):
                    cmd = ctrl(path, sim.end, sim.previous, sim.travel, sim.blind)
                else:
                    cmd = ctrl(path, sim.end, sim.previous)
                    if cmd is None:
                        cmd = np.array([0.06, 0.0] if sim.blind < 0.35 else [0.0, 0.8])
                _, _, reason = sim.step(cmd)
            r = sim.result(reason)
            r["difficulty"] = diffs[k % 3]
            rows.append(r)
        done = [r for r in rows if r["complete"]]
        res = dict(episodes=len(rows), complete=len(done),
                   complete_pct=round(100 * len(done) / len(rows), 1),
                   mean_progress_pct=round(float(np.mean([r["progress_pct"] for r in rows])), 1),
                   lap_s=round(float(np.mean([r["sim_sec"] for r in done])), 1) if done else None,
                   wheel_events=round(float(np.mean([r["wheel_line_events"] for r in rows])), 2),
                   by_difficulty={d: f"{sum(r['complete'] for r in rows if r['difficulty']==d)}/"
                                     f"{sum(1 for r in rows if r['difficulty']==d)}" for d in diffs},
                   wall_s=round(time.time() - t0, 1))
        results[name] = res
        print(f"{name:16} {res['complete']:3}/{res['episodes']}  mean {res['mean_progress_pct']:5.1f}%  "
              f"lap {res['lap_s']}  {res['by_difficulty']}  ({res['wall_s']} s)", flush=True)
    Path(a.out).write_text(json.dumps(results, indent=1))


if __name__ == "__main__":
    main()
