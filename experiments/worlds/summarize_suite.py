#!/usr/bin/env python3
"""Summarise a bench run over a suite, broken down by map features (dev tool).

  python3 experiments/worlds/summarize_suite.py BENCH_OUT_DIR [SUITE_INDEX_CSV ...] [--png out.png]

Joins every <run>/result.json in BENCH_OUT_DIR with the suite index rows (by map file name)
and prints completion rate / mean progress / lap time per difficulty, hill count, mirror and
obstacle count, plus the worst maps. Infrastructure errors are counted separately.
"""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bench_out")
    ap.add_argument("index", nargs="*")
    ap.add_argument("--png")
    a = ap.parse_args()
    meta = {}
    for idx in a.index:
        for row in csv.DictReader(open(idx)):
            meta[Path(row["path"]).name] = row
    runs = []
    for f in sorted(Path(a.bench_out).glob("*/result.json")):
        r = json.loads(f.read_text())
        name = Path(r["map"]).name
        m = meta.get(name, {"difficulty": "official" if "line_track" in name else "?", "hills": "?",
                            "mirror": "?", "obstacles": "?"})
        runs.append((name, r, m))
    if not runs:
        raise SystemExit("no result.json found")
    infra = [x for x in runs if str(x[1].get("reason", "")).startswith("error")]
    good = [x for x in runs if x not in infra]

    def line(label, items):
        n = len(items)
        done = [r for _, r, _ in items if r.get("complete")]
        prog = sum(r.get("progress_pct") or 0 for _, r, _ in items) / max(n, 1)
        lap = sum(r["lap_sec"] for r in done) / len(done) if done else None
        coll = sum(r.get("collision_count") or 0 for _, r, _ in items)
        return f"{label:22} {len(done):3}/{n:<3} {100*len(done)/max(n,1):5.1f}%  mean {prog:5.1f}%  " \
               f"lap {('%.1f s' % lap) if lap else '   -  ':>8}  hits {coll}"

    print(f"runs {len(runs)}  (infrastructure errors {len(infra)} excluded)")
    print(line("ALL", good))
    for key in ("difficulty", "hills", "mirror", "loop_corners", "detours", "bars", "gaps", "small_loops", "obstacles"):
        groups = defaultdict(list)
        for x in good:
            groups[x[2].get(key, "?")].append(x)
        for value in sorted(groups, key=str):
            print(line(f"  {key}={value}", groups[value]))
    worst = sorted(good, key=lambda x: x[1].get("progress_pct") or 0)[:10]
    print("worst maps:")
    for name, r, _ in worst:
        print(f"  {name:40} {r.get('progress_pct')}%  {str(r.get('reason'))[:45]}")
    if a.png:
        import matplotlib
        matplotlib.use("Agg")
        from matplotlib import pyplot as plt
        keys_ = ("difficulty", "hills", "mirror", "loop_corners", "detours", "gaps")
        fig, axes = plt.subplots(2, 3, figsize=(15, 8), layout="constrained")
        for ax, key in zip(axes.flat, keys_):
            groups = defaultdict(list)
            for x in good:
                groups[x[2].get(key, "?")].append(x)
            keys = sorted(groups, key=str)
            rate = [100 * sum(bool(r.get("complete")) for _, r, _ in groups[k]) / len(groups[k]) for k in keys]
            prog = [sum(r.get("progress_pct") or 0 for _, r, _ in groups[k]) / len(groups[k]) for k in keys]
            xs = range(len(keys))
            ax.bar([i - 0.2 for i in xs], rate, width=0.4, label="completed %")
            ax.bar([i + 0.2 for i in xs], prog, width=0.4, label="mean progress %")
            ax.set_xticks(list(xs), [f"{k}\n(n={len(groups[k])})" for k in keys])
            ax.set_ylim(0, 100)
            ax.set_title(f"by {key}")
            ax.legend(fontsize=8)
        fig.savefig(a.png, dpi=100)
        print(f"wrote {a.png}")


if __name__ == "__main__":
    main()
