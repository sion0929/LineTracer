#!/usr/bin/env python3
"""Completion table per profile: official track and tile maps, with Fisher one-sided p vs ref."""
import json, sys
from math import comb
from pathlib import Path
D = Path(sys.argv[1])
res = [json.loads(p.read_text()) for p in sorted(D.glob("*/result.json"))]


def fisher_greater(a, b, c, d):
    """one-sided p that group1 (a ok / b fail) is better than group2 (c ok / d fail)."""
    n1, n2, k = a + b, c + d, a + c
    tot = comb(n1 + n2, k)
    return sum(comb(n1, i) * comb(n2, k - i) for i in range(a, min(n1, k) + 1)) / tot


prof = sorted({r["profile"] for r in res}, key=lambda p: ["ref", "near320", "dual", "dual640"].index(p) if p in ["ref", "near320", "dual", "dual640"] else 9)
out = {}
for p in prof:
    rr = [r for r in res if r["profile"] == p]
    off = [r for r in rr if r["map"].endswith("worlds/line_track.sdf")]
    til = [r for r in rr if "tiles60" in r["map"]]
    laps = [r["lap_sec"] for r in off if r["complete"]]
    out[p] = dict(off_done=sum(r["complete"] for r in off), off_n=len(off),
                  off_lap=sum(laps) / len(laps) if laps else None,
                  til_done=sum(r["complete"] for r in til), til_n=len(til),
                  til_prog=sum(r["progress_pct"] for r in til) / max(1, len(til)),
                  lat=sum(r["lat_mean"] for r in rr if r["complete"]) / max(1, sum(r["complete"] for r in rr)) * 100)
ref = out.get("ref")
for p, o in out.items():
    pv = None
    if ref and p != "ref":
        a, b = o["off_done"] + o["til_done"], o["off_n"] + o["til_n"] - o["off_done"] - o["til_done"]
        c, d = ref["off_done"] + ref["til_done"], ref["off_n"] + ref["til_n"] - ref["off_done"] - ref["til_done"]
        o["p_better"] = round(fisher_greater(a, b, c, d), 3)
        o["p_worse"] = round(fisher_greater(c, d, a, b), 3)
    print(p, o)
json.dump(out, open(D / "summary.json", "w"), indent=1)
