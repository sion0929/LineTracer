#!/usr/bin/env python3
r"""Score BEV post-filters offline by replaying perception logs through LineMap.

Usage (repository root):
  tools/pylf experiments/eval/log_filter_eval.py BATCH_DIR [BATCH_DIR ...] \
      --filters none hill_lateral_0.15 hill_lateral_0.20 hill_lateral_0.25 hill_lateral_0.30 \
      --out OUT.md

Each run is replayed frame by frame (d['used'] only) into a fresh ``LineMap`` for
every filter.  The resulting cells are moved from odom to world with the run's
``job.json`` ``odom_origin`` and scored per hill zone by ``hill_zone_eval``.
Nothing is simulated; no ground truth enters the map.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "experiments/eval"))

from hill_zone_eval import ALL_ZONES, load_map, pool_metrics, run_key, score_map_cells  # noqa: E402
from line_follower.line_map import C0, RES, LineMap  # noqa: E402
from line_follower.perception.log import load as load_log  # noqa: E402


def cell_y(meta):
    """Cell-centre y in the robot frame for each BEV column (col 0 = left, +y)."""
    grid = meta["grid"]
    res = float(grid["res_m"])
    y_half = float(grid["y_half_m"])
    width = int(grid["shape"][1])
    return y_half - (np.arange(width) + 0.5) * res


def make_filter(name):
    """(line_bool (BH, BW), slope_state int, meta) -> filtered line_bool."""
    if name == "none":
        return lambda line, slope, meta: line
    if name.startswith("hill_lateral_"):
        limit = float(name[len("hill_lateral_"):])

        def hill_lateral(line, slope, meta):
            if slope not in (1, 2, 3):
                return line
            out = line.copy()
            out[:, np.abs(cell_y(meta)) > limit] = False
            return out

        return hill_lateral
    raise ValueError(f"Unknown filter {name!r}")


def rebuild(d, meta, filt, active):
    """Feed every used frame through a fresh LineMap; return the map."""
    line_map = LineMap()
    line, valid, pose = d["line"], d["valid"], d["pose"]
    slope = d["slope"]
    for i in np.nonzero(active)[0]:
        filtered = filt(line[i], int(slope[i]), meta)
        line_map.update(filtered.astype(np.uint8) * 255,
                        valid[i].astype(np.uint8) * 255,
                        pose[i])
    return line_map


def map_cells_world(line_map, origin):
    """grid > 0 cells -> odom (x, y) -> world (x, y)."""
    i, j = np.nonzero(line_map.grid > 0)
    x = (j - C0) * RES
    y = (i - C0) * RES
    ox, oy, oth = (float(v) for v in origin)
    c, s = math.cos(oth), math.sin(oth)
    return np.column_stack((ox + c * x - s * y, oy + s * x + c * y))


def discover(batch):
    """Run directories with a perception log, plus skip reasons."""
    runs, skipped = [], []
    runs_dir = batch / "runs"
    if not runs_dir.is_dir():
        skipped.append({"batch": batch.name, "run": None,
                        "reason": f"missing {runs_dir}"})
        return runs, skipped
    for run in sorted(runs_dir.iterdir()):
        if not run.is_dir():
            continue
        if not (run / "perception_log").is_dir():
            skipped.append({"batch": batch.name, "run": run.name,
                            "reason": "missing perception_log"})
            continue
        job_path = run / "job.json"
        if not job_path.is_file():
            skipped.append({"batch": batch.name, "run": run.name,
                            "reason": "missing job.json"})
            continue
        runs.append(run)
    return runs, skipped


def evaluate(batch_dirs, filters):
    results = {name: {} for name in filters}       # filter -> (batch, key) -> zone metrics
    live = {}                                      # (batch, key) -> zone metrics
    origins = {}
    skipped = []
    run_ids = []
    frames = 0
    started = time.perf_counter()

    for batch in batch_dirs:
        batch = Path(batch)
        runs, run_skips = discover(batch)
        skipped.extend(run_skips)
        for run in runs:
            rid = (batch.name, run_key(run.name))
            try:
                job = json.loads((run / "job.json").read_text())
                map_meta, ramps, _ = load_map(job)
                log_meta, d = load_log(run / "perception_log")
            except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
                skipped.append({"batch": batch.name, "run": run.name,
                                "reason": str(error)})
                continue
            if not d or "slope" not in d:
                skipped.append({"batch": batch.name, "run": run.name,
                                "reason": "format-1 log (no slope field)"})
                continue
            if np.all(d["slope"] == -1):
                skipped.append({"batch": batch.name, "run": run.name,
                                "reason": "all slope -1 (unknown hill state)"})
                continue

            origin = job.get("odom_origin", [0.0, 0.0, 0.0])
            origins[rid] = origin
            active = d["used"].astype(bool)
            frames += int(active.sum())
            run_ids.append(rid)

            for name in filters:
                line_map = rebuild(d, log_meta, make_filter(name), active)
                xy = map_cells_world(line_map, origin)
                results[name][rid] = score_map_cells(xy, map_meta, ramps)

            npz = run / "map_world.npz"
            if npz.is_file():
                try:
                    with np.load(npz) as data:
                        xy = data["xy"].reshape(-1, 2).astype(float)
                    live[rid] = score_map_cells(xy, map_meta, ramps)
                except (OSError, ValueError, KeyError) as error:
                    skipped.append({"batch": batch.name, "run": run.name,
                                    "reason": f"map_world.npz: {error}"})

    runtime = time.perf_counter() - started

    pooled = {name: pool_metrics([results[name][rid] for rid in run_ids
                                  if rid in results[name]])
              for name in filters}

    base = "none"
    paired = {}
    if base in results:
        for name in filters:
            if name == base:
                continue
            paired[name] = {}
            for zone in ALL_ZONES:
                differences, wins, maps = [], 0, 0
                for rid in run_ids:
                    if rid not in results[base] or rid not in results[name]:
                        continue
                    score = results[name][rid][zone]["precision"]
                    baseline = results[base][rid][zone]["precision"]
                    if score is None or baseline is None:
                        continue
                    differences.append(score - baseline)
                    maps += 1
                    if score >= baseline:
                        wins += 1
                paired[name][zone] = {
                    "mean_precision_diff": float(np.mean(differences)) if differences else None,
                    "wins": wins,
                    "maps": maps,
                }

    sanity = None
    if base in results:
        subset = [rid for rid in run_ids if rid in live and rid in results[base]]
        if subset:
            offline = pool_metrics([results[base][rid] for rid in subset])["all"]["precision"]
            online = pool_metrics([live[rid] for rid in subset])["all"]["precision"]
            sanity = {
                "runs": len(subset),
                "offline_all_precision": offline,
                "live_all_precision": online,
                "difference": (None if offline is None or online is None
                               else offline - online),
            }

    return {
        "filters": list(filters),
        "base": base,
        "runs": [{"batch": batch, "run": key} for batch, key in run_ids],
        "pooled": pooled,
        "paired": paired,
        "sanity": sanity,
        "skipped": skipped,
        "frames": frames,
        "runtime_s": runtime,
    }


def _pr(precision, recall):
    left = "—" if precision is None else f"{precision:.3f}"
    right = "—" if recall is None else f"{recall:.3f}"
    return f"{left} / {right}"


def markdown(report):
    lines = ["# Offline BEV filter evaluation", "",
             f"Frames replayed: {report['frames']} · runs: {len(report['runs'])}",
             "",
             "## Pooled precision / recall", "",
             "| Filter | " + " | ".join(ALL_ZONES) + " | cells |",
             "|---|" + "---:|" * (len(ALL_ZONES) + 1)]
    for name in report["filters"]:
        zones = report["pooled"][name]
        cells = " | ".join(_pr(zones[z]["precision"], zones[z]["recall"]) for z in ALL_ZONES)
        lines.append(f"| `{name}` | {cells} | {zones['all']['cells']} |")

    lines += ["", f"## Paired precision difference vs `{report['base']}`", "",
              "| Filter | " + " | ".join(ALL_ZONES) + " |",
              "|---|" + "---:|" * len(ALL_ZONES)]
    if report["paired"]:
        for name in report["filters"]:
            if name not in report["paired"]:
                continue
            cells = []
            for zone in ALL_ZONES:
                item = report["paired"][name][zone]
                if item["mean_precision_diff"] is None:
                    cells.append("—")
                else:
                    cells.append(f"{item['mean_precision_diff']:+.3f} "
                                 f"({item['wins']}/{item['maps']})")
            lines.append(f"| `{name}` | " + " | ".join(cells) + " |")
    else:
        lines.append("| — | " + " | ".join("—" for _ in ALL_ZONES) + " |")

    lines += ["", "## Sanity check: `none` offline vs live `map_world.npz`", ""]
    sanity = report["sanity"]
    if sanity is None:
        lines.append("_No run had both `none` results and `map_world.npz`._")
    else:
        lines += [f"Runs compared: {sanity['runs']}", "",
                  "| | pooled `all` precision |",
                  "|---|---:|",
                  f"| offline rebuild (`none`) | "
                  f"{_pr(sanity['offline_all_precision'], None).split(' / ')[0]} |",
                  f"| live map (`map_world.npz`) | "
                  f"{_pr(sanity['live_all_precision'], None).split(' / ')[0]} |",
                  f"| difference | {sanity['difference']:+.3f} |"]

    if report["skipped"]:
        lines += ["", "## Skipped", "",
                  "| Batch | Run | Reason |", "|---|---|---|"]
        for item in report["skipped"]:
            lines.append(f"| {item['batch']} | {item['run'] or '—'} | {item['reason']} |")

    lines += ["", f"Runtime: {report['runtime_s']:.1f} s"]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("batch_dirs", type=Path, nargs="+", metavar="BATCH_DIR")
    parser.add_argument("--filters", nargs="+", default=[
        "none", "hill_lateral_0.15", "hill_lateral_0.20",
        "hill_lateral_0.25", "hill_lateral_0.30"])
    parser.add_argument("--out", type=Path, required=True, help="markdown output path")
    args = parser.parse_args()

    filters = list(dict.fromkeys(args.filters))
    for name in filters:
        make_filter(name)

    report = evaluate(args.batch_dirs, filters)
    text = markdown(report)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text)
    args.out.with_suffix(".json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
