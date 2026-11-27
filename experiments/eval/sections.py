#!/usr/bin/env python3
"""Grading-sheet sections on the official track: per-section reach and mean |error| <= 2 cm (dev).

Section ends are taken from the graded run of 2026-11-25 (cumulative times, 64.084 s lap) mapped
to arc position s with a reference bench run that has the same lap time (same controller):
  python3 experiments/eval/sections.py calibrate REF_RUN_DIR        -> experiments/eval/sections.json
  python3 experiments/eval/sections.py score RUNS_DIR [RUNS_DIR...]  -> table per profile
Error = |lat| of the robot origin (wheel axle centre) to the centre line, telemetry column "lat".
"""
import glob, json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from telemetry import COLUMNS  # noqa: E402
GRADED_SPLITS = [4.24, 9.092, 13.72, 20.268, 27.068, 33.072, 36.612, 39.976, 43.148, 47.752, 54.364, 58.544, 62.096, 64.084]
GRADED_ACC = "OOOXOOOOOOXOOX"
NAMES = ["start+ramp up", "ramp down", "arc", "right corner", "2 arcs", "2 right corners", "arc", "right corner",
         "long straight", "crossing 1 + arc", "loop", "arc + crossing 2", "3 arcs", "finish"]


def load(run):
    t = np.load(Path(run) / "telemetry.npz")
    c = {k: t["data"][:, i] for i, k in enumerate(COLUMNS)}
    moved = np.nonzero(c["progress"] > 0.02)[0]
    t0 = c["t"][moved[0]] if len(moved) else c["t"][0]
    return c, t0


def calibrate(run):
    c, t0 = load(run)
    lap = c["t"][-1] - t0
    ends = [float(np.interp(ts * lap / GRADED_SPLITS[-1] + t0, c["t"], c["progress"])) for ts in GRADED_SPLITS]
    json.dump({"ends_s": ends, "ref_run": str(run), "ref_lap": float(lap)}, open(HERE / "sections.json", "w"), indent=1)
    for k, (e, n) in enumerate(zip(ends, NAMES), 1):
        print(f"S{k:2d} ends at s={e:6.2f} m  {n}")


def score_run(run, ends):
    c, _ = load(run)
    r = json.load(open(Path(run) / "result.json"))
    out = []
    lo = 0.0
    for e in ends:
        m = (c["s"] >= lo) & (c["s"] < e) & (c["progress"] >= lo)
        reached = c["progress"].max() >= lo + 0.8 * (e - lo)
        err = float(np.mean(np.abs(c["lat"][m]))) * 100 if m.sum() > 5 else np.nan
        out.append((reached, err))
        lo = e
    return r, out


def score(dirs):
    ends = json.load(open(HERE / "sections.json"))["ends_s"]
    per = {}
    for d in dirs:
        for run in sorted(glob.glob(f"{d}/*/")):
            if not (Path(run) / "telemetry.npz").exists() or "line_track" not in json.load(open(Path(run) / "job.json"))["map"]:
                continue
            r, out = score_run(run, ends)
            if r.get("reason", "").startswith("error:"):
                continue
            per.setdefault(r["profile"], []).append(out)
    print("section accuracy pass rate (mean |error| <= 2 cm, among runs that reached it); graded run =", GRADED_ACC)
    print("profile        n  " + " ".join(f"S{k:<4d}" for k in range(1, 15)) + "  reach+acc pts")
    for p, runs in per.items():
        cells, pts = [], []
        for k in range(14):
            ok = [e <= 2.0 for reached, e in (run[k] for run in runs) if reached and np.isfinite(e)]
            cells.append(f"{(sum(ok) / len(ok) if ok else float('nan')):5.2f}")
        for run in runs:
            reach = sum((1 if k < 8 else 2) for k in range(14) if run[k][0])
            acc = sum(2 for k in range(14) if run[k][0] and np.isfinite(run[k][1]) and run[k][1] <= 2.0)
            pts.append(reach + acc)
        print(f"{p:12s} {len(runs):3d}  " + " ".join(cells) + f"  {np.mean(pts):5.1f}/48")
    print("mean error [cm] per section:")
    for p, runs in per.items():
        print(f"{p:12s}      " + " ".join(f"{np.nanmean([run[k][1] for run in runs]):5.2f}" for k in range(14)))


if __name__ == "__main__":
    if sys.argv[1] == "calibrate":
        calibrate(sys.argv[2])
    else:
        score(sys.argv[2:])
