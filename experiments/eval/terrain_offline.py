#!/usr/bin/env python3
"""Flat IPM (node pitch logic) vs line-width height profile (terrain.py), on recorded frames (dev tool).

Input: raw.npz files from shadow_route.py with SHADOW_RAW=1. For each frame both methods
produce a BEV line mask; line pixels go to the world with the TRUE pose and their distance to
the true centre line is measured. Split by frame type (flat / hill = |pitch_f| > 3 deg /
rocking) and by range (near < 0.5 m, mid 0.5-0.85, far >= 0.85 m; far needs LF_X_FAR > 0.85).

  LF_X_FAR=1.15 python3 experiments/eval/terrain_offline.py RUN_DIR ... --out out.json
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "line_follower"))
sys.path.insert(0, str(ROOT / "experiments" / "oracle"))
from line_follower import terrain as T  # noqa: E402
from line_follower import vision as V  # noqa: E402
import gt  # noqa: E402

REST = math.radians(0.33)


def imu_pitch(imu, t):
    ok = ~np.isnan(imu[:, 0])
    if not ok.any():
        return None
    a = imu[ok]
    return float(a[int(np.argmin(np.abs(a[:, 0] - t))), 1])


def pixel_errors(line, valid, truth, route):
    rows, cols = np.nonzero((line > 0) & (valid > 0))
    if len(rows) < 15:
        return None
    if len(rows) > 800:
        sel = np.linspace(0, len(rows) - 1, 800).astype(int)
        rows, cols = rows[sel], cols[sel]
    x, y = V.bev_to_robot(cols, rows)
    X, Y, yaw = truth[0], truth[1], truth[2]
    c, s = math.cos(yaw), math.sin(yaw)
    w = np.column_stack((X + x * c - y * s, Y + x * s + y * c))
    near = np.linalg.norm(route - np.array([X, Y]), axis=1) < 1.6
    if near.sum() < 2:
        return None
    return np.asarray(x), gt.seg_distance(w, route[near])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--hold", type=float, default=0.4, help="reuse the last profile this long [s]")
    ap.add_argument("--bend-z", type=float, default=0.015, help="hyb2: height change ahead that counts as terrain [m]")
    a = ap.parse_args()
    methods = ("flat", "profile", "hyb1", "hyb2")
    ranges = (("near", 0.0, 0.5), ("mid", 0.5, 0.85), ("far", 0.85, 9.0))
    acc = {}
    stats = {"frames": 0, "profile_ok": 0, "bend": 0}
    for run in a.runs:
        z = dict(np.load(run / "raw.npz"))
        route = z["route"][:, :2]
        last, last_t = None, -9.0
        for i in range(len(z["ts"])):
            truth, pf, ts = z["truth"][i], z["pitch_f"][i], z["ts"][i]
            kind = "hill" if abs(pf) > math.radians(3.0) else ("rock" if z["rock"][i] >= 0.08 else "flat")
            stats["frames"] += 1
            th = REST if kind == "hill" else (imu_pitch(z["imu"][i], ts) or REST)
            flat_line, flat_valid = V.line_mask(z["gray"][i], th, V.X_FAR)
            prof, _ = T.estimate(z["gray"][i], REST)
            if prof is not None:
                last, last_t = prof, ts
                stats["profile_ok"] += 1
                stats["bend"] += prof.b is not None
            elif last is not None and ts - last_t < a.hold:
                prof = last
            if prof is None:
                prof_line, prof_valid = flat_line, flat_valid       # fall back to the node's choice
            else:
                prof_line, prof_valid = T.line_mask_profile(z["gray"][i], REST, prof, V.X_FAR)
            bend_ahead = (prof is not None and prof.b is not None
                          and abs(prof.ds) * max(0.0, V.X_FAR - prof.b) > a.bend_z)
            hyb1 = (prof_line, prof_valid) if kind == "hill" else (flat_line, flat_valid)
            hyb2 = (prof_line, prof_valid) if (kind == "hill" or bend_ahead) else (flat_line, flat_valid)
            for name, (ln, vd) in (("flat", (flat_line, flat_valid)), ("profile", (prof_line, prof_valid)),
                                   ("hyb1", hyb1), ("hyb2", hyb2)):
                r = pixel_errors(ln, vd, truth, route)
                if r is None:
                    continue
                x, d = r
                for rn, lo, hi in ranges:
                    m = (x >= lo) & (x < hi)
                    acc.setdefault((name, kind, rn), []).extend(d[m].tolist())
                    acc.setdefault((name, "all", rn), []).extend(d[m].tolist())
    print(stats)
    res = {"stats": stats}
    print(f"{'method':8} {'frames':6} " + "  ".join(f"{rn:>4} p50/p90 cm (n)" for rn, *_ in ranges))
    for kind in ("all", "flat", "rock", "hill"):
        for name in methods:
            row = []
            for rn, *_ in ranges:
                v = np.array(acc.get((name, kind, rn), []))
                if len(v):
                    p = (round(float(np.percentile(v, 50)) * 100, 2), round(float(np.percentile(v, 90)) * 100, 2), len(v))
                    row.append(f"{p[0]:6.2f}/{p[1]:6.2f} ({p[2]:6})")
                    res[f"{name}|{kind}|{rn}"] = p
                else:
                    row.append(" " * 22)
            print(f"{name:8} {kind:6} " + "  ".join(row))
    a.out.write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
