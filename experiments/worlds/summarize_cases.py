#!/usr/bin/env python3
"""Per-situation results for the targeted case suite (gen_cases.py).

For every bench run on a case track: complete or not, and where it failed relative to the
case's feature window (before / inside / after). Prints one row per group and per case name.

  python3 experiments/worlds/summarize_cases.py BENCH_OUTPUT_DIR [--by-name] [--csv out.csv]
"""
import argparse
import collections
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", type=Path)
    ap.add_argument("--by-name", action="store_true")
    ap.add_argument("--csv", type=Path)
    a = ap.parse_args()
    rows = []
    for res in sorted(a.runs.glob("*/result.json")):
        job = json.loads((res.parent / "job.json").read_text())
        r = json.loads(res.read_text())
        mp = Path(job["map"])
        if not mp.exists() and str(mp).startswith("/ws/"):          # ran inside the lab container
            mp = ROOT / str(mp)[4:]
        meta = json.loads(mp.with_suffix(".json").read_text())
        case = meta.get("case")
        if not case:
            continue
        s0, s1 = case["feature_s"]
        prog = r.get("progress_m") or 0.0
        if r.get("complete"):
            where = "done"
        elif str(r.get("reason", "")).startswith("error"):
            where = "error"
        elif prog < s0 - 0.05:
            where = "before"
        elif prog <= s1 + 0.30:
            where = "feature"
        else:
            where = "after"
        rows.append((case["group"], case["name"] + ("_m" if case["mirror"] else ""), job.get("profile", ""), where,
                     prog, case["length_m"], r.get("lap_sec"), r.get("reason")))
    by = collections.defaultdict(list)
    for row in rows:
        key = (row[0], row[2]) if not a.by_name else (f"{row[0]}/{row[1]}", row[2])
        by[key].append(row)
    print(f"{'group':34} {'profile':10} {'done':>7}  fail@feature  before  after  error")
    tot = collections.Counter()
    for (g, p), rs in sorted(by.items()):
        c = collections.Counter(x[3] for x in rs)
        tot.update(c)
        print(f"{g:34} {p:10} {c['done']:3}/{len(rs):<3}  {c['feature']:12}  {c['before']:6}  {c['after']:5}  {c['error']:5}")
    n = sum(tot.values())
    print(f"{'TOTAL':34} {'':10} {tot['done']:3}/{n:<3}  {tot['feature']:12}  {tot['before']:6}  {tot['after']:5}  {tot['error']:5}")
    if a.csv:
        with a.csv.open("w") as f:
            f.write("group,case,profile,where,progress_m,length_m,lap_s,reason\n")
            for row in rows:
                f.write(",".join(str(v) for v in row) + "\n")


if __name__ == "__main__":
    main()
