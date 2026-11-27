#!/usr/bin/env python3
"""Completion per profile (official track / tile maps) with one-sided Fisher p vs the first profile.
  python3 experiments/eval/bench_summary.py RUNS_DIR [ref_profile]
"""
import glob, json, sys
from math import comb
D = sys.argv[1]
rs = [json.load(open(f)) for f in sorted(glob.glob(f"{D}/*/result.json"))]
rs = [r for r in rs if not r.get("reason", "").startswith("error:")]     # infrastructure errors
order = []
for r in rs:
    if r["profile"] not in order:
        order.append(r["profile"])
ref = sys.argv[2] if len(sys.argv) > 2 else order[0]


def fg(a, b, c, d):
    n1, n2, k = a + b, c + d, a + c
    return sum(comb(n1, i) * comb(n2, k - i) for i in range(a, min(n1, k) + 1)) / comb(n1 + n2, k)


agg = {}
for p in order:
    rr = [r for r in rs if r["profile"] == p]
    off = [r for r in rr if r["map"].endswith("worlds/line_track.sdf")]
    til = [r for r in rr if not r["map"].endswith("worlds/line_track.sdf")]   # any test map suite
    laps = [r["lap_sec"] for r in off if r["complete"]]
    agg[p] = dict(official=f"{sum(r['complete'] for r in off)}/{len(off)}",
                  lap=round(sum(laps) / len(laps), 2) if laps else None,
                  tiles=f"{sum(r['complete'] for r in til)}/{len(til)}",
                  tiles_prog=round(sum(r["progress_pct"] for r in til) / max(1, len(til)), 1),
                  ok=sum(r["complete"] for r in rr), n=len(rr))
for p, a in agg.items():
    if p != ref:
        r = agg[ref]
        a["p_better"] = round(fg(a["ok"], a["n"] - a["ok"], r["ok"], r["n"] - r["ok"]), 3)
        a["p_worse"] = round(fg(r["ok"], r["n"] - r["ok"], a["ok"], a["n"] - a["ok"]), 3)
    print(p, json.dumps(a, ensure_ascii=False))
json.dump(agg, open(f"{D}/completion.json", "w"), indent=1)
