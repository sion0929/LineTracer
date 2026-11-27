#!/usr/bin/env python3
"""Check the line-width height profile (terrain.py) on Isaac Sim frames with true depth (dev tool).

Isaac frames (experiments/isaac/capture.py, Codex-4) place the robot on the centre line with the
body pitched to the local slope; the camera is mounted level (pitch 0) 0.103 m above the
ground. For every true line pixel (classes == 1) the depth along the camera axis is known, so
two predictions are compared on the same pixel ray:

  flat     ground plane under the robot (what the node does on a slope: rest pitch, z = 0)
  profile  the height profile fitted from line widths in the rendered RGB (terrain.estimate)

Error = horizontal distance between predicted and true ground point. Frames are split by the
true terrain ahead (max |height| of the line within 1.2 m vs the robot's plane > 1 cm).

  python experiments/eval/terrain_isaac.py DATASET_DIR --out out.json
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "line_follower"))
from line_follower import terrain as T  # noqa: E402
from line_follower import vision as V  # noqa: E402


def ray_t_profile(k, a, prof, iters=6):
    """Depth t along rays (down-slope k, forward factor a) that meet z = prof(x)."""
    t = V.CAM_H / k
    for _ in range(iters):
        x = V.CAM_X + t * a
        z = prof(x)
        t = np.where(k > 1e-4, (V.CAM_H - z) / np.maximum(k, 1e-4), np.nan)
    return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("data", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--max-frames", type=int, default=100000)
    a = ap.parse_args()
    files = sorted(a.data.glob("*/seed_*/*.npz"))[:a.max_frames]
    bins = (("near", 0.0, 0.5), ("mid", 0.5, 0.85), ("far", 0.85, 1.2))
    acc, n = {}, {"frames": 0, "profile": 0, "terrain": 0}
    for f in files:
        z = np.load(f)
        cls, depth = z["classes"], z["depth_m"]
        vv, uu = np.nonzero((cls == 1) & np.isfinite(depth) & (depth > 0))
        vv, uu = vv[vv > V.CY + 2], uu[vv > V.CY + 2]
        if len(vv) < 50:
            continue
        n["frames"] += 1
        k = (vv - V.CY) / V.FX                    # theta = 0: down component per unit depth
        lat = (uu - V.CX) / V.FX
        t_gt = depth[vv, uu].astype(float)
        x_gt = V.CAM_X + t_gt
        z_gt = V.CAM_H - t_gt * k
        sel = (x_gt > 0.3) & (x_gt < 1.2)
        terrain = bool(sel.any() and np.max(np.abs(z_gt[sel])) > 0.01)
        n["terrain"] += terrain
        kind = "terrain" if terrain else "plane"
        rgb = z["rgb"]
        gray = (0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]).astype(np.uint8)
        prof, _ = T.estimate(gray, 0.0)
        n["profile"] += prof is not None
        preds = {"flat": V.CAM_H / k}
        preds["profile"] = ray_t_profile(k, 1.0, prof) if prof is not None else preds["flat"]
        for name, t in preds.items():
            err = np.abs(t - t_gt) * np.sqrt(1.0 + lat * lat)
            for bn, lo, hi in bins:
                m = (x_gt >= lo) & (x_gt < hi) & np.isfinite(err)
                for key in ((name, kind, bn), (name, "all", bn)):
                    acc.setdefault(key, []).extend(err[m][::4].tolist())
    print(n)
    res = {"counts": n}
    print(f"{'method':8} {'frames':8} " + "  ".join(f"{b:>4} p50/p90 cm (n)" for b, *_ in bins))
    for kind in ("all", "plane", "terrain"):
        for name in ("flat", "profile"):
            row = []
            for bn, *_ in bins:
                v = np.array(acc.get((name, kind, bn), []))
                if len(v):
                    p = (round(float(np.percentile(v, 50)) * 100, 2), round(float(np.percentile(v, 90)) * 100, 2), len(v))
                    res[f"{name}|{kind}|{bn}"] = p
                    row.append(f"{p[0]:6.2f}/{p[1]:6.2f} ({p[2]:6})")
                else:
                    row.append(" " * 22)
            print(f"{name:8} {kind:8} " + "  ".join(row))
    a.out.write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
