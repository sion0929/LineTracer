#!/usr/bin/env python3
"""Attribute each failed bench run to the map feature at the failure point (dev tool).

For every run: find the arc length s where the run ended (max progress) and look up what the
generated map has within +-0.35 m of that s along the centre line: hill (on/approach/crest/foot),
square/arc corner (heading change), corner circle, detour, gap, crossing/stub bar, loop ellipse
crossing. Prints counts and a per-run table.

  python3 experiments/eval/failure_locus.py BENCH_OUT MAP_DIR [MAP_DIR ...] [--csv out.csv]
"""
import argparse
import csv
import glob
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np


def features_at(meta, s_fail, win=0.35):
    c = np.asarray(meta["centerline"], float)
    arc = np.asarray(meta["arc_m"], float)
    i = int(np.clip(np.searchsorted(arc, s_fail), 1, len(arc) - 2))
    lo, hi = np.searchsorted(arc, s_fail - win), np.searchsorted(arc, s_fail + win)
    seg = c[max(lo, 0):min(hi + 1, len(c))]
    tags = []
    z = seg[:, 2]
    if z.max() > 0.005:
        dz = np.diff(z)
        if (dz > 1e-4).any() and (dz < -1e-4).any():
            tags.append("hill_crest")
        elif (dz > 1e-4).any():
            tags.append("hill_up")
        elif (dz < -1e-4).any():
            tags.append("hill_down")
        else:
            tags.append("hill_top")
    elif len(c) and np.abs(np.diff(c[max(lo - 20, 0):min(hi + 20, len(c)), 2])).max(initial=0) > 1e-4:
        tags.append("hill_near")
    h = np.unwrap(np.arctan2(np.diff(seg[:, 1]), np.diff(seg[:, 0])))
    if len(h) > 2 and abs(h[-1] - h[0]) > math.radians(45):
        tags.append("corner")
    p = c[i, :2]
    for lc in meta.get("loop_corners", []):
        if math.dist(lc["vertex"], p) < 0.45:
            tags.append("corner_circle")
    for d in meta.get("detours", []):
        x0, x1 = d["x"]
        if x0 - 0.2 <= p[0] <= x1 + 0.2 and abs(p[1] - c[i, 1]) < 1 and any(
                abs(p[1] - y) < 0.5 for y in [c[i, 1]]):
            tags.append("detour")
    for g0, g1 in meta.get("gaps_arc_m", []):
        if g0 - win <= s_fail <= g1 + win:
            tags.append("gap")
    for m in meta.get("markings", []):
        m = np.asarray(m, float)
        if len(m) <= 3 and np.min(np.linalg.norm(m[:, :2] - p, axis=1)) < 0.3:
            tags.append("bar_or_stub_or_T")
        elif len(m) > 10 and np.min(np.linalg.norm(m[:, :2] - p, axis=1)) < 0.25:
            tags.append("loop_ellipse")
    if s_fail < 0.6:
        tags.append("start")
    if s_fail > arc[-1] - 0.6:
        tags.append("finish")
    return tags or ["straight"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bench_out")
    ap.add_argument("map_dirs", nargs="+")
    ap.add_argument("--csv")
    a = ap.parse_args()
    metas = {}
    for d in a.map_dirs:
        for j in glob.glob(str(Path(d) / "*.json")):
            metas[Path(j).stem] = json.loads(Path(j).read_text())
    rows, tally, primary = [], Counter(), Counter()
    for f in sorted(glob.glob(str(Path(a.bench_out) / "*/result.json"))):
        r = json.loads(Path(f).read_text())
        name = Path(r["map"]).stem
        if r.get("complete") or str(r.get("reason", "")).startswith("error") or name not in metas:
            continue
        s = r.get("max_progress_m") or r.get("progress_m") or 0.0
        tags = features_at(metas[name], s)
        tally.update(set(tags))
        order = ["start", "hill_up", "hill_crest", "hill_down", "hill_top", "corner_circle", "detour",
                 "gap", "bar_or_stub_or_T", "loop_ellipse", "corner", "hill_near", "finish", "straight"]
        primary[next(t for t in order if t in tags)] += 1
        rows.append(dict(map=name, s=round(s, 2), reason=str(r.get("reason"))[:30], tags="+".join(tags)))
    n = len(rows)
    print(f"failed runs analysed: {n}")
    print("primary cause (first matching in priority order):")
    for k, v in primary.most_common():
        print(f"  {k:18} {v:3}  {100*v/max(n,1):5.1f}%")
    print("any tag present:")
    for k, v in tally.most_common():
        print(f"  {k:18} {v:3}")
    if a.csv:
        with open(a.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)


if __name__ == "__main__":
    main()
