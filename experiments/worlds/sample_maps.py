#!/usr/bin/env python3
"""Randomly pick N maps from the test suites (dev tool).

  python3 experiments/worlds/sample_maps.py 6                       # random seed, printed + saved
  python3 experiments/worlds/sample_maps.py 6 --seed 1234           # reproduce a sample
  python3 experiments/worlds/sample_maps.py 10 --suites hills60,stress50 --out /tmp/s
  python3 experiments/worlds/sample_maps.py 6 --with-obstacles      # also draw from obstacles100

Suites (experiments/worlds/suites/<name>/index.csv, TEAM_GUIDE 4-1). Default:
general200, official_like50, hills60, stress50. obstacles100 only with --with-obstacles or
when named in --suites.

Picking is balanced by default: the N draws are spread as evenly as possible over the chosen
suites (remainder suites drawn at random), then maps are drawn without replacement inside each
suite. --mode pool draws uniformly over all maps of the chosen suites instead.

Missing map files are regenerated one by one with gen_world.generate(), replaying the exact
per-map draws of make_suite.py (same recipe as the suite was made with), and every regenerated
map is checked against its index.csv row (length, hills, mirror, boxes, ...). A mismatch is an
error, never a silent substitute.

Writes into --out (default experiments/results/random/samples/<time>_seed<seed>/):
  maps.txt     map paths relative to the workspace root, one per line (bench.py input:
               python3 experiments/bench.py $(cat maps.txt) ...)
  maps.csv     suite + the suite index.csv row of every picked map
  sample.json  seed, suites, mode, N and the command line (to reproduce)
"""
import argparse
import contextlib
import csv
import fcntl
import json
import random
import secrets
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SUITES_DIR = HERE / "suites"
sys.path.insert(0, str(HERE))

DEFAULT_SUITES = ("general200", "official_like50", "hills60", "stress50")
# make_suite.py arguments each suite was generated with (TEAM_GUIDE 4-1). hills60 is not in the
# guide; its recipe was recovered by replaying make_suite on the committed index.csv (all 60 rows
# match). Keys left out use make_suite.py's defaults.
MAKE_SUITE_DEFAULTS = dict(seed0=1000, hills="1-4", obstacles="0", mirror_rate=0.5, small_loops="0",
                           loop_corners="0-2", detours="0-1", stubs="0-2", crosses="0-2", gaps="0-3",
                           difficulty="easy,normal,hard")
RECIPES = {
    "general200": dict(count=200),
    "obstacles100": dict(count=100, seed0=3000, obstacles="1-3"),
    "official_like50": dict(count=50, seed0=5000, difficulty="hard,normal", hills="2-4",
                            loop_corners="1-2", detours="1", stubs="1-2", crosses="0-2", gaps="1-3"),
    "stress50": dict(count=50, seed0=7000, difficulty="hard", hills="3-4", loop_corners="2", detours="1",
                     stubs="2", crosses="2", gaps="3", small_loops="1-2"),
    "hills60": dict(count=60, seed0=9000, hills="2-4", loop_corners="0-1", detours="0", stubs="0",
                    crosses="0", gaps="0"),
}
CHECK_COLUMNS = ("difficulty", "hills", "mirror", "obstacles", "length_m", "max_ramp_m", "min_corner_r_m",
                 "square_corners", "loop_corners", "detours", "small_loops", "bars", "gaps")


def _range(text):
    lo, hi = (str(text).split("-") * 2)[:2]
    return int(lo), int(hi)


def recipe(suite):
    if suite not in RECIPES:
        raise ValueError(f"no make_suite recipe for suite {suite!r} (known: {sorted(RECIPES)})")
    return dict(MAKE_SUITE_DEFAULTS, **RECIPES[suite])


def load_index(suite):
    path = SUITES_DIR / suite / "index.csv"
    if not path.is_file():
        raise FileNotFoundError(f"{path} missing; make the suite first (TEAM_GUIDE 4-1)")
    with path.open() as f:
        return list(csv.DictReader(f))


def index_row(meta, sdf):
    """Same feature row make_suite.py writes (kept identical so rows can be compared)."""
    ramps = meta.get("ramps", [meta["ramp"]])
    return dict(seed=meta["seed"], difficulty=meta["difficulty"], hills=len(ramps),
                mirror=int(bool(meta.get("mirror", False))), obstacles=len(meta["obstacles"]),
                length_m=round(meta["arc_m"][-1], 3),
                max_ramp_m=round(max(r["height_m"] for r in ramps), 3),
                min_corner_r_m=round(min([r for r in meta["corner_radii_m"] if r] or [0]), 3),
                square_corners=sum(1 for r in meta["corner_radii_m"] if not r),
                loop_corners=len(meta.get("loop_corners", [])),
                detours=len(meta.get("detours", [])),
                small_loops=len(meta.get("small_loops", [])),
                bars=max(0, len(meta["markings"]) - 2 - sum(1 for m in meta["markings"] if len(m) > 10)),
                gaps=len(meta.get("gaps_arc_m", [])),
                path=str(sdf))


def _same(a, b):
    try:
        return abs(float(a) - float(b)) < 1e-9
    except ValueError:
        return str(a) == str(b)


@contextlib.contextmanager
def _locked(directory):
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / ".generate.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)


def ensure_map(suite, row):
    """Regenerate one suite map (SDF + STL + JSON) if any of its files is missing.

    Replays make_suite.py's per-map draws for this seed, then checks the result against the
    index row. Returns True when the map was generated now."""
    sdf = ROOT / row["path"]
    if all(sdf.with_suffix(ext).is_file() for ext in (".sdf", ".stl", ".json")):
        return False
    import gen_world

    r = recipe(suite)
    seed, k = int(row["seed"]), int(row["seed"]) - r["seed0"]
    if not 0 <= k < r["count"]:
        raise ValueError(f"{suite}: seed {seed} outside the recipe range")
    pick = random.Random(seed * 31 + 7)                       # make_suite.py, same draw order
    diffs = r["difficulty"].split(",")
    diff = diffs[k % len(diffs)]
    hills = pick.randint(*_range(r["hills"]))
    mirror = pick.random() < r["mirror_rate"]
    boxes = pick.randint(*_range(r["obstacles"]))
    nl, nc, ng = (pick.randint(*_range(r[key])) for key in ("small_loops", "crosses", "gaps"))
    nlc, nd, ns = (pick.randint(*_range(r[key])) for key in ("loop_corners", "detours", "stubs"))
    expected = SUITES_DIR / suite / "maps" / f"{diff}_{seed}{'_m' if mirror else ''}.sdf"
    if expected.resolve() != sdf.resolve():
        raise ValueError(f"{suite}: recipe gives {expected.name} but index.csv has {sdf.name}")
    with _locked(sdf.parent):
        if all(sdf.with_suffix(ext).is_file() for ext in (".sdf", ".stl", ".json")):
            return False
        meta = None
        for attempt in range(boxes + 1):
            try:
                meta = gen_world.generate(seed, diff, boxes - attempt, sdf, hills, mirror, nl, nc, ng,
                                          nlc, nd, ns)
                break
            except ValueError:
                continue
        if meta is None:
            raise RuntimeError(f"{suite}: gen_world could not generate {sdf.name}")
    got = index_row(meta, row["path"])
    bad = [c for c in CHECK_COLUMNS if c in row and not _same(row[c], got[c])]
    if bad:
        for ext in (".sdf", ".stl", ".json"):
            sdf.with_suffix(ext).unlink(missing_ok=True)
        raise RuntimeError(f"{suite}/{sdf.name}: regenerated map differs from index.csv in {bad} "
                           f"({[(row[c], got[c]) for c in bad]}); files removed")
    return True


def sample(n, suites, seed, mode="balanced"):
    """Return [(suite, row)] for n maps drawn with random.Random(seed)."""
    rng = random.Random(seed)
    pools = {s: load_index(s) for s in suites}
    total = sum(len(p) for p in pools.values())
    if not 1 <= n <= total:
        raise ValueError(f"N must be in 1..{total} for suites {list(suites)}")
    if mode == "pool":
        everything = [(s, row) for s in suites for row in pools[s]]
        return rng.sample(everything, n)
    if mode != "balanced":
        raise ValueError(f"unknown mode {mode!r}")
    counts = {s: 0 for s in suites}
    left = n
    while left:
        open_ = [s for s in suites if counts[s] < len(pools[s])]
        share, extra = divmod(left, len(open_))
        for s in open_:
            counts[s] += min(share, len(pools[s]) - counts[s])
        for s in rng.sample(open_, extra):
            if counts[s] < len(pools[s]):
                counts[s] += 1
        left = n - sum(counts.values())
    picked = [(s, row) for s in suites for row in rng.sample(pools[s], counts[s])]
    rng.shuffle(picked)
    return picked


def write_outputs(out, picked, info):
    out.mkdir(parents=True, exist_ok=True)
    (out / "maps.txt").write_text("".join(row["path"] + "\n" for _, row in picked))
    columns = ["order", "suite"] + [c for c in picked[0][1].keys()]
    with (out / "maps.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        for i, (suite, row) in enumerate(picked, 1):
            writer.writerow(dict(row, order=i, suite=suite))
    (out / "sample.json").write_text(json.dumps(info, indent=2, ensure_ascii=False) + "\n")


def run(n, suites=DEFAULT_SUITES, seed=None, mode="balanced", out=None, generate=True, quiet=False):
    """Pick n maps, regenerate missing ones and write maps.txt/maps.csv/sample.json.

    Returns (out_dir, seed, [(suite, row)]). Importable by the other eval tools."""
    seed = secrets.randbelow(2 ** 31) if seed is None else int(seed)
    picked = sample(n, list(suites), seed, mode)
    out = Path(out) if out else ROOT / "experiments/results/random/samples" / \
        f"{time.strftime('%Y%m%d-%H%M%S')}_seed{seed}"
    made = 0
    if generate:
        for suite, row in picked:
            made += ensure_map(suite, row)
    info = dict(seed=seed, n=n, suites=list(suites), mode=mode, generated_now=made,
                maps=[row["path"] for _, row in picked],
                reproduce=f"python3 experiments/worlds/sample_maps.py {n} --seed {seed} "
                          f"--suites {','.join(suites)} --mode {mode}",
                created=time.strftime("%Y-%m-%d %H:%M:%S"))
    write_outputs(out, picked, info)
    if not quiet:
        print(f"sample seed: {seed}   (reproduce: {info['reproduce']})")
        for i, (suite, row) in enumerate(picked, 1):
            print(f"  {i:2d}. {suite:16} {Path(row['path']).name:24} {row['difficulty']:6} hills {row['hills']} "
                  f"mirror {row['mirror']} circ {row.get('loop_corners', '?')} det {row.get('detours', '?')} "
                  f"bars {row.get('bars', '?')} gaps {row.get('gaps', '?')} boxes {row['obstacles']}")
        print(f"generated now: {made}   wrote {out}/maps.txt, maps.csv, sample.json")
    return out, seed, picked


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("n", type=int, help="number of maps")
    ap.add_argument("--suites", default=",".join(DEFAULT_SUITES), help="comma list of suite names")
    ap.add_argument("--with-obstacles", action="store_true", help="add obstacles100 to the suites")
    ap.add_argument("--seed", type=int, help="sampling seed (default: random, printed and saved)")
    ap.add_argument("--mode", choices=("balanced", "pool"), default="balanced")
    ap.add_argument("--out", type=Path, help="output directory")
    ap.add_argument("--no-generate", action="store_true", help="do not regenerate missing map files")
    a = ap.parse_args()
    suites = [s for s in a.suites.split(",") if s]
    if a.with_obstacles and "obstacles100" not in suites:
        suites.append("obstacles100")
    try:
        run(a.n, suites, a.seed, a.mode, a.out, not a.no_generate)
    except (ValueError, FileNotFoundError, RuntimeError) as error:
        ap.error(str(error))


if __name__ == "__main__":
    main()
