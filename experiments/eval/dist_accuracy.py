#!/usr/bin/env python3
"""Map-cell accuracy by the distance at which each cell was first recorded (official runs, dev).
  python3 experiments/eval/dist_accuracy.py RUNS_DIR [tol_m]
"""
import glob, json, struct, sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
ROOT = Path(__file__).resolve().parents[2]
b = (ROOT / "src/line_follower_bringup/models/track/meshes/track.stl").read_bytes()
n = struct.unpack("<I", b[80:84])[0]
v = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("x", "<u2")]))["v"].reshape(-1, 3, 3)[:, :, :2]
rng = np.random.default_rng(0)
pts = []
for tri in v.astype(float):
    r = rng.random((60, 2)); f = r.sum(1) > 1; r[f] = 1 - r[f]
    pts.append(tri[0] + r[:, :1] * (tri[1] - tri[0]) + r[:, 1:] * (tri[2] - tri[0]))
tree = cKDTree(np.vstack(pts))
tol = float(sys.argv[2]) if len(sys.argv) > 2 else 0.03
bins = np.arange(0.3, 1.61, 0.1)
res = {}
for d in sorted(glob.glob(f"{sys.argv[1]}/*/")):
    j = json.load(open(d + "job.json"))
    if not j["map"].endswith("line_track.sdf") or not Path(d + "map_world.npz").exists():
        continue
    z = np.load(d + "map_world.npz")
    e = tree.query(z["xy"])[0]; dd = z["dist"]
    r = res.setdefault(j["profile"], [[0, 0] for _ in bins])
    for k, b0 in enumerate(bins):
        m = (dd >= b0) & (dd < b0 + 0.1)
        r[k][0] += int((e[m] < tol).sum()); r[k][1] += int(m.sum())
print(f"cells within {tol*100:.0f} cm by first-recorded distance (count per profile in brackets)")
print("dist  " + "  ".join(f"{p:>14}" for p in res))
for k, b0 in enumerate(bins):
    print(f"{b0:.1f}  " + "  ".join(f"{(r[k][0]/r[k][1]*100 if r[k][1] > 500 else float('nan')):6.1f}% ({r[k][1]:6d})" for r in res.values()))
json.dump({p: {f"{b0:.1f}": r[k] for k, b0 in enumerate(bins)} for p, r in res.items()}, open(f"{sys.argv[1]}/dist_accuracy.json", "w"))
