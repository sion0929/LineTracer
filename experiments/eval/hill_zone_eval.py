#!/usr/bin/env python3
r"""Score perception line maps on the hill (ramp) zones.

Usage (repository root):
  tools/pylf experiments/eval/hill_zone_eval.py BATCH_DIR [BATCH_DIR ...] \
      --base PROFILE --out OUT.md

Each BATCH_DIR is one profile (its folder name is the profile name) with
``runs/<run>/map_world.npz`` and ``runs/<run>/job.json``.  Ground truth and
segment distance come from :mod:`perception_map_eval`; ramp geometry copies
``ground_height()`` from ``experiments/isaac/drive_core.py`` (heading form and
the legacy x/y breakpoint form).
"""
import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "experiments/eval"))
import perception_map_eval as pme  # noqa: E402

OFFICIAL_SDF = ROOT / "src/line_follower_bringup/worlds/line_track.sdf"
ZONES = ("up", "deck", "down", "approach", "flat")
ALL_ZONES = ZONES + ("all",)
APPROACH_REACH = 0.40
APPROACH_WIDTH = 0.10
EPS = 1e-12


def official_ramps():
    """Legacy-form ramp used by the official track (along = x)."""
    return [dict(x=[0.66, 1.10, 1.54, 1.98], y=[0.0, 0.66], height_m=0.06)]


def ramp_list(meta):
    ramps = meta.get("ramps")
    if ramps:
        return list(ramps) if isinstance(ramps, (list, tuple)) else [ramps]
    ramp = meta.get("ramp")
    if ramp:
        return list(ramp) if isinstance(ramp, (list, tuple)) else [ramp]
    return []


def _frame(ramp, x, y):
    """Return along, |across|, breakpoints a/b/c/d and half width, as ground_height."""
    if "heading" in ramp:
        dx, dy = x - ramp["x0"], y - ramp["y0"]
        c, s = math.cos(ramp["heading"]), math.sin(ramp["heading"])
        along = dx * c + dy * s
        across = np.abs(-dx * s + dy * c)
        up, deck, down = ramp["up"], ramp["deck"], ramp["down"]
        a, b, c, d = 0.0, up, up + deck, up + deck + down
        half = ramp.get("width", 0.66) / 2.0
    else:
        along = x
        a, b, c, d = ramp["x"]
        y0, y1 = ramp["y"]
        across = np.abs(y - (y0 + y1) / 2.0)
        half = (y1 - y0) / 2.0
    return along, across, a, b, c, d, half


def classify_zones(ramps, points):
    """Classify world points into up/deck/down/approach/flat (first ramp wins)."""
    pts = np.asarray(points, float).reshape(-1, 2)
    x, y = pts[:, 0], pts[:, 1]
    zones = np.full(len(pts), "flat", dtype="<U8")
    free = np.ones(len(pts), bool)
    for ramp in ramps:
        along, across, a, b, c, d, half = _frame(ramp, x, y)
        inside = across <= half + EPS

        mask = free & inside & (along >= a - EPS) & (along < b - EPS)
        zones[mask], free[mask] = "up", False
        mask = free & inside & (along >= b - EPS) & (along < c - EPS)
        zones[mask], free[mask] = "deck", False
        mask = free & inside & (along >= c - EPS) & (along <= d + EPS)
        zones[mask], free[mask] = "down", False

        near = across <= half + APPROACH_WIDTH + EPS
        before = (along >= a - APPROACH_REACH - EPS) & (along < a - EPS)
        after = (along > d + EPS) & (along <= d + APPROACH_REACH + EPS)
        mask = free & near & (before | after)
        zones[mask], free[mask] = "approach", False
    return zones


def score_map_cells(xy, meta, ramps):
    """Precision/recall/cell counts per hill zone for one map."""
    ink, samples, _ = pme.ground_truth(meta)
    width = float(meta.get("line_width_m", 0.02))
    tolerance = width / 2.0 + 0.01
    xy = np.asarray(xy, float).reshape(-1, 2)

    if len(xy):
        correct = pme.segment_distance(xy, ink) <= tolerance + EPS
    else:
        correct = np.zeros(0, bool)

    cell_zones = classify_zones(ramps, xy)
    sample_zones = classify_zones(ramps, samples)
    if len(xy) and len(samples):
        hits = cKDTree(xy).query(samples)[0] <= tolerance + EPS
    else:
        hits = np.zeros(len(samples), bool)

    metrics = {}
    for zone in ZONES:
        cells = cell_zones == zone
        count = int(cells.sum())
        sample_mask = sample_zones == zone
        sample_count = int(sample_mask.sum())
        metrics[zone] = {
            "precision": float(correct[cells].sum()) / count if count else None,
            "recall": float(hits[sample_mask].sum()) / sample_count if sample_count else None,
            "cells": count,
            "correct_cells": int(correct[cells].sum()),
            "sample_points": sample_count,
            "hit_points": int(hits[sample_mask].sum()),
        }

    count, sample_count = len(xy), len(samples)
    metrics["all"] = {
        "precision": float(correct.sum()) / count if count else None,
        "recall": float(hits.sum()) / sample_count if sample_count else None,
        "cells": count,
        "correct_cells": int(correct.sum()),
        "sample_points": sample_count,
        "hit_points": int(hits.sum()),
    }
    return metrics


def pool_metrics(metrics_list):
    """Pool zone counts across maps for one profile."""
    pooled = {}
    for zone in ALL_ZONES:
        correct = sum(m[zone]["correct_cells"] for m in metrics_list)
        cells = sum(m[zone]["cells"] for m in metrics_list)
        hits = sum(m[zone]["hit_points"] for m in metrics_list)
        samples = sum(m[zone]["sample_points"] for m in metrics_list)
        pooled[zone] = {
            "precision": correct / cells if cells else None,
            "recall": hits / samples if samples else None,
            "cells": cells,
            "correct_cells": correct,
            "sample_points": samples,
            "hit_points": hits,
        }
    return pooled


def run_key(name):
    """Drop the leading zero-padded counter from a run folder name."""
    match = re.match(r"^\d+_(.*)$", name)
    return match.group(1) if match else name


def _map_path(raw):
    path = Path(raw)
    if not path.is_absolute():
        return ROOT / path
    if path.parts[:2] == ("/", "ws"):
        return ROOT.joinpath(*path.parts[2:])
    return path


def load_map(job):
    raw = job["map"]
    if raw == "official":
        meta = pme.official_metadata(OFFICIAL_SDF)
        return meta, official_ramps(), True
    path = _map_path(raw)
    meta = json.loads(path.read_text())
    return meta, ramp_list(meta), False


def evaluate(batch_dirs, base):
    profiles, skipped = {}, []
    for batch in batch_dirs:
        batch = Path(batch)
        profile = batch.name
        runs = {}
        runs_dir = batch / "runs"
        if not runs_dir.is_dir():
            skipped.append({"profile": profile, "run": None, "reason": f"missing {runs_dir}"})
            profiles[profile] = runs
            continue
        for run in sorted(runs_dir.iterdir()):
            if not run.is_dir():
                continue
            npz = run / "map_world.npz"
            if not npz.is_file():
                skipped.append({"profile": profile, "run": run.name, "reason": "missing map_world.npz"})
                continue
            job_path = run / "job.json"
            if not job_path.is_file():
                skipped.append({"profile": profile, "run": run.name, "reason": "missing job.json"})
                continue
            try:
                job = json.loads(job_path.read_text())
                meta, ramps, official = load_map(job)
                with np.load(npz) as data:
                    xy = data["xy"].reshape(-1, 2).astype(float)
                metrics = score_map_cells(xy, meta, ramps)
            except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
                skipped.append({"profile": profile, "run": run.name, "reason": str(error)})
                continue
            runs[run_key(run.name)] = {"zones": metrics, "run": run.name,
                                       "official": official}
        profiles[profile] = runs

    if base not in profiles:
        raise SystemExit(f"--base profile {base!r} not found among {sorted(profiles)}")

    pooled = {name: pool_metrics([item["zones"] for item in runs.values()])
              for name, runs in profiles.items()}

    paired = {}
    for name, runs in profiles.items():
        if name == base:
            continue
        paired[name] = {}
        for zone in ALL_ZONES:
            differences, wins, maps = [], 0, 0
            for key, item in runs.items():
                if key not in profiles[base]:
                    continue
                score = item["zones"][zone]["precision"]
                baseline = profiles[base][key]["zones"][zone]["precision"]
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
    return {"base": base, "profiles": pooled, "paired": paired, "skipped": skipped}


def _format_pr(precision, recall):
    left = "—" if precision is None else f"{precision:.3f}"
    right = "—" if recall is None else f"{recall:.3f}"
    return f"{left} / {right}"


def markdown(report):
    rows = ["# Hill zone evaluation", "",
            "## Pooled precision / recall", "",
            "| Profile | " + " | ".join(ALL_ZONES) + " | cells |",
            "|---|" + "---:|" * (len(ALL_ZONES) + 1)]
    for name in sorted(report["profiles"]):
        zones = report["profiles"][name]
        cells = " | ".join(_format_pr(zones[zone]["precision"], zones[zone]["recall"])
                           for zone in ALL_ZONES)
        rows.append(f"| {name} | {cells} | {zones['all']['cells']} |")

    rows += ["", f"## Paired precision difference vs `{report['base']}`", "",
             "| Profile | " + " | ".join(ALL_ZONES) + " |",
             "|---|" + "---:|" * len(ALL_ZONES)]
    for name in sorted(report["paired"]):
        cells = []
        for zone in ALL_ZONES:
            item = report["paired"][name][zone]
            if item["mean_precision_diff"] is None:
                cells.append("—")
            else:
                cells.append(f"{item['mean_precision_diff']:+.3f} "
                             f"({item['wins']}/{item['maps']})")
        rows.append(f"| {name} | " + " | ".join(cells) + " |")
    if not report["paired"]:
        rows.append("| — | " + " | ".join("—" for _ in ALL_ZONES) + " |")

    if report["skipped"]:
        rows += ["", f"Skipped {len(report['skipped'])} run(s) without usable map data."]
    return "\n".join(rows) + "\n"


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("batch_dirs", type=Path, nargs="+", metavar="BATCH_DIR")
    parser.add_argument("--base", required=True, help="profile name to compare against")
    parser.add_argument("--out", type=Path, required=True, help="markdown output path")
    args = parser.parse_args()

    report = evaluate(args.batch_dirs, args.base)
    text = markdown(report)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text)
    if str(args.out) != "/dev/null":
        args.out.with_suffix(".json").write_text(
            json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
