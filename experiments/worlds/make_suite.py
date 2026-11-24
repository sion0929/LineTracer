#!/usr/bin/env python3
"""Generate a large, varied test suite of worlds (dev tool).

Every map mixes difficulty (corner radii / square corners / crossing loops), number of
hills, mirroring (left/right turns swapped), official-style small circles at corners
(loop_corners, 90 deg self-crossing), connected crossings that loop round and rejoin the
road (detours), crossing bars, one-sided branches (stubs), breaks in the painted line
(gaps), optional teardrop loops (ambiguous, off by default) and optionally boxes. Seeds start at 1000 so the
fixed evaluation sets (seeds 0-39) are never reused for tuning.

  python3 experiments/worlds/make_suite.py --name hills200 --count 200
  python3 experiments/worlds/make_suite.py --name obstacles100 --count 100 --obstacles 1-3

Writes experiments/worlds/suites/<name>/:
  maps/*.sdf|.stl|.json   the worlds (regenerable, git-ignored)
  list.txt                map paths, one per line (feed to bench.py)
  index.csv               seed, difficulty, hills, mirror, obstacles, length, max ramp height
  sheet_XX.png            20 previews per page
"""
import argparse
import csv
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_world  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", required=True)
    ap.add_argument("--count", type=int, default=100)
    ap.add_argument("--seed0", type=int, default=1000)
    ap.add_argument("--hills", default="1-4", help="range of requested hills, e.g. 1-4")
    ap.add_argument("--obstacles", default="0", help="range of boxes, e.g. 0 or 1-3")
    ap.add_argument("--mirror-rate", type=float, default=0.5)
    ap.add_argument("--small-loops", default="0", help="range of teardrop loops (ambiguous)")
    ap.add_argument("--loop-corners", default="0-2", help="range of official-style corner circles")
    ap.add_argument("--detours", default="0-1", help="range of connected crossings")
    ap.add_argument("--stubs", default="0-2", help="range of one-sided branches")
    ap.add_argument("--crosses", default="0-2", help="range of crossing bars")
    ap.add_argument("--gaps", default="0-3", help="range of breaks in the painted line")
    ap.add_argument("--difficulty", default="easy,normal,hard", help="comma list cycled per map")
    ap.add_argument("--no-sheets", action="store_true")
    a = ap.parse_args()

    rng_range = lambda txt: tuple(int(v) for v in (txt.split("-") * 2)[:2])  # noqa: E731
    h_lo, h_hi = rng_range(a.hills)
    o_lo, o_hi = rng_range(a.obstacles)
    l_lo, l_hi = rng_range(a.small_loops)
    c_lo, c_hi = rng_range(a.crosses)
    g_lo, g_hi = rng_range(a.gaps)
    lc_lo, lc_hi = rng_range(a.loop_corners)
    d_lo, d_hi = rng_range(a.detours)
    s_lo, s_hi = rng_range(a.stubs)
    diffs = a.difficulty.split(",")
    out = ROOT / "experiments/worlds/suites" / a.name
    maps = out / "maps"
    maps.mkdir(parents=True, exist_ok=True)
    rows, metas = [], []
    for k in range(a.count):
        seed = a.seed0 + k
        pick = random.Random(seed * 31 + 7)
        diff = diffs[k % len(diffs)]
        hills = pick.randint(h_lo, h_hi)
        mirror = pick.random() < a.mirror_rate
        boxes = pick.randint(o_lo, o_hi)
        nl, nc, ng = pick.randint(l_lo, l_hi), pick.randint(c_lo, c_hi), pick.randint(g_lo, g_hi)
        nlc, nd, ns = pick.randint(lc_lo, lc_hi), pick.randint(d_lo, d_hi), pick.randint(s_lo, s_hi)
        sdf = maps / f"{diff}_{seed}{'_m' if mirror else ''}.sdf"
        for attempt in range(boxes + 1):
            try:
                meta = gen_world.generate(seed, diff, boxes - attempt, sdf, hills, mirror, nl, nc, ng,
                                          nlc, nd, ns)
                break
            except ValueError:
                continue
        ramps = meta.get("ramps", [meta["ramp"]])
        rows.append(dict(seed=seed, difficulty=diff, hills=len(ramps), mirror=int(mirror),
                         obstacles=len(meta["obstacles"]), length_m=round(meta["arc_m"][-1], 3),
                         max_ramp_m=round(max(r["height_m"] for r in ramps), 3),
                         min_corner_r_m=round(min([r for r in meta["corner_radii_m"] if r] or [0]), 3),
                         square_corners=sum(1 for r in meta["corner_radii_m"] if not r),
                         loop_corners=len(meta.get("loop_corners", [])),
                         detours=len(meta.get("detours", [])),
                         small_loops=len(meta.get("small_loops", [])),
                         bars=max(0, len(meta["markings"]) - 2 - sum(1 for m in meta["markings"] if len(m) > 10)),
                         gaps=len(meta.get("gaps_arc_m", [])),
                         path=str(sdf.relative_to(ROOT))))
        metas.append(meta)
    (out / "list.txt").write_text("".join(r["path"] + "\n" for r in rows))
    with (out / "index.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    if not a.no_sheets:
        import matplotlib
        matplotlib.use("Agg")
        from matplotlib import pyplot as plt
        for page in range(0, len(metas), 20):
            fig, axes = plt.subplots(4, 5, figsize=(15, 12), layout="constrained")
            for ax in axes.flat:
                ax.axis("off")
            for ax, meta, row in zip(axes.flat, metas[page:page + 20], rows[page:page + 20]):
                ax.axis("on")
                gen_world.draw(ax, meta)
                ax.set_title(f"{row['difficulty']} {row['seed']}{' M' if row['mirror'] else ''} · "
                             f"hill{row['hills']} circ{row['loop_corners']} det{row['detours']} "
                             f"bar{row['bars']} gap{row['gaps']} box{row['obstacles']}", fontsize=8)
            fig.savefig(out / f"sheet_{page // 20:02d}.png", dpi=80)
            plt.close(fig)
    n = len(rows)
    print(f"{out.relative_to(ROOT)}: {n} maps · hills {sum(r['hills'] for r in rows)/n:.2f} avg "
          f"(max {max(r['hills'] for r in rows)}) · mirrored {sum(r['mirror'] for r in rows)} · "
          f"boxes {sum(r['obstacles'] for r in rows)} · corner circles {sum(r['loop_corners'] for r in rows)} · "
          f"detours {sum(r['detours'] for r in rows)} · bars+stubs {sum(r['bars'] for r in rows)} · "
          f"gaps {sum(r['gaps'] for r in rows)} · teardrops {sum(r['small_loops'] for r in rows)}")


if __name__ == "__main__":
    main()
