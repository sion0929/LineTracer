#!/usr/bin/env python3
"""How wrong is the BEV projection while the body rocks, and does a better pitch fix it? (dev tool)

Input: raw.npz recorded by experiments/oracle/shadow_route.py with SHADOW_RAW=1 (camera frames,
IMU pitch history, true pose incl. pitch). For each frame the rule line mask is projected with
several pitch sources, the line pixels are put into the world with the TRUE pose, and their
distance to the true centre line is measured, split into near (x < 0.5 m) and far (x >= 0.5 m)
pixels and into steady / rocking frames.

pitch sources:  rest      fixed REST pitch (0)
                imu@dt    IMU pitch at image stamp + dt (dt in ms; the node uses dt = 0)
                truth     true body pitch (sign checked both ways)

  python3 experiments/eval/pitch_offline.py RUN_DIR [RUN_DIR ...] --out out.json
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
from line_follower import vision  # noqa: E402
import gt  # noqa: E402


def imu_pitch(imu, t):
    ok = ~np.isnan(imu[:, 0])
    if not ok.any():
        return None
    a = imu[ok]
    k = int(np.argmin(np.abs(a[:, 0] - t)))
    return float(a[k, 1])


def errors(gray, theta, truth, route, x_max=vision.X_FAR):
    line, valid = vision.line_mask(gray, theta, x_max)
    rows, cols = np.nonzero((line > 0) & (valid > 0))
    if len(rows) < 20:
        return None
    if len(rows) > 600:
        sel = np.linspace(0, len(rows) - 1, 600).astype(int)
        rows, cols = rows[sel], cols[sel]
    x, y = vision.bev_to_robot(cols, rows)
    X, Y, yaw = truth[0], truth[1], truth[2]
    c, s = math.cos(yaw), math.sin(yaw)
    w = np.column_stack((X + x * c - y * s, Y + x * s + y * c))
    near = np.linalg.norm(route - np.array([X, Y]), axis=1) < 1.3
    d = gt.seg_distance(w, route[near]) if near.sum() >= 2 else np.full(len(w), np.nan)
    return np.asarray(x), d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--rock", type=float, default=0.08)
    a = ap.parse_args()
    dts = (-0.10, -0.05, -0.03, 0.0, 0.03, 0.05, 0.10)
    sources = ["rest"] + [f"imu@{int(dt * 1000)}" for dt in dts] + ["truth+", "truth-"]
    acc = {(src, grp, rng): [] for src in sources for grp in ("steady", "rocking")
           for rng in ("near", "far")}
    n = {"steady": 0, "rocking": 0}
    for run in a.runs:
        z = dict(np.load(run / "raw.npz"))         # NpzFile re-decompresses on every access
        route = z["route"][:, :2]
        for i in range(len(z["ts"])):
            truth = z["truth"][i]
            flat = abs(z["pitch_f"][i]) < math.radians(3.0)
            if not flat:
                continue                      # hills: plane assumption is wrong anyway
            grp = "rocking" if z["rock"][i] >= a.rock else "steady"
            n[grp] += 1
            for src in sources:
                if src == "rest":
                    th = math.radians(0.33)          # follower_node.REST_PITCH
                elif src.startswith("imu@"):
                    th = imu_pitch(z["imu"][i], z["ts"][i] + int(src[4:]) / 1000.0)
                    if th is None:
                        continue
                else:
                    th = truth[3] if src == "truth+" else -truth[3]
                r = errors(z["gray"][i], th, truth, route)
                if r is None:
                    continue
                x, d = r
                acc[(src, grp, "near")].extend(d[x < 0.5].tolist())
                acc[(src, grp, "far")].extend(d[x >= 0.5].tolist())
    res = {"frames": n}
    print(f"frames: {n}")
    print(f"{'source':10} " + "  ".join(f"{g}/{r} p50/p90 cm" for g in ("steady", "rocking") for r in ("near", "far")))
    for src in sources:
        row = []
        for g in ("steady", "rocking"):
            for rng in ("near", "far"):
                v = np.array(acc[(src, g, rng)])
                v = v[~np.isnan(v)]
                p = (round(float(np.percentile(v, 50)) * 100, 2), round(float(np.percentile(v, 90)) * 100, 2), len(v)) if len(v) else None
                res[f"{src}|{g}|{rng}"] = p
                row.append(f"{p[0]:5.2f}/{p[1]:5.2f} ({p[2]:5})" if p else "   -   ")
        print(f"{src:10} " + "  ".join(row))
    a.out.write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
