#!/usr/bin/env python3
r"""Score truth-aligned perception maps from the repository root.

Bench (requires ROS/Gazebo and the generated suite SDF and STL assets):
  LAB_NAME=line_follower_lab python3 experiments/bench.py \
    experiments/worlds/suites/decoy1000_test/maps/decoy_09*.sdf \
    --follower experiments/oracle/run_oracle_map.py --off 0.12 \
    --output experiments/results/perception_runs
Report (NumPy, SciPy, Matplotlib):
  python3 experiments/eval/perception_map_eval.py \
    experiments/results/perception_runs experiments/results/perception_report

Precision tests map cell centres against continuous GT centrelines, within half the
paint width plus 1 cm (2 cm at the standard width). Recall tests 5 mm centreline
samples against true-positive map cells using the same tolerance. Denominators are
samples within 1.0 m or 0.5 m of the truth trajectory polyline in XY. This is a
proximity proxy for visibility. Main includes non-decoy markings. At shared
route/decoy points, route takes precedence. First-observation bands independently
measure coverage of the same in-view ink by cells first logged in that band, so
band recalls can overlap. Aggregates pool counts across runs, including partial drives.
Scores use the logged occupancy layer (xy). The restored layer is logged separately.
Confidence uses first-logged confidence and empirical precision per 0.2-wide bin.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
from scipy.spatial import cKDTree


ROOT = Path(__file__).resolve().parents[2]
STEP = 0.005
BANDS = ("0-0.5", "0.5-0.9", "0.9-1.3", "1.3+")
RADII = (1.0, 0.5)
CLASSES = ("all", "main", "decoy")


def segments(paths):
    pieces = []
    for path in paths:
        if len(path) >= 2:
            xy = np.asarray(path, float)[:, :2]
            pieces.extend(zip(xy[:-1], xy[1:]))
    return np.asarray(pieces, float).reshape(-1, 2, 2)


def segment_distance(points, lines):
    """Exact projection, in bounded chunks to limit temporary array size."""
    if not len(lines):
        return np.full(len(points), np.inf)
    a, b = lines.transpose(1, 0, 2)
    delta = b - a
    length2 = np.maximum(np.sum(delta ** 2, axis=1), 1e-20)
    distances = np.empty(len(points))
    for start in range(0, len(points), 256):
        points_chunk = points[start:start + 256]
        offsets = points_chunk[:, None, :] - a
        fraction = np.clip(np.einsum("ijk,jk->ij", offsets, delta) / length2, 0, 1)
        residual = offsets - fraction[..., None] * delta
        distances[start:start + len(points_chunk)] = np.sqrt(
            np.einsum("ijk,ijk->ij", residual, residual).min(axis=1))
    return distances


def official_metadata(world):
    """Derive centrelines from the checked-in official mesh's paired strip triangles."""
    official_worlds = [ROOT / "src" / package / "worlds/line_track.sdf"
                       for package in ("line_follower_bringup", "line_follower")]
    if world.resolve() not in official_worlds:
        raise FileNotFoundError(world.with_suffix(".json"))
    mesh = ROOT / "src/line_follower_bringup/models/track/meshes/track.stl"
    dtype = np.dtype([("normal", "<f4", 3), ("xyz", "<f4", (3, 3)), ("attribute", "<u2")])
    triangles = np.frombuffer(mesh.read_bytes(), dtype=dtype, offset=84)["xyz"]
    first, second = triangles[::2], triangles[1::2]
    if (len(first) != len(second) or not np.array_equal(first[:, 0], second[:, 0])
            or not np.array_equal(first[:, 2], second[:, 1])):
        raise ValueError(f"{mesh}: expected paired strip triangles")
    quad = np.stack((first[:, 0], first[:, 1], first[:, 2], second[:, 2]), axis=1)
    quad = quad[:, :, :2].astype(float)
    edges = np.roll(quad, -1, axis=1)
    width_error = np.abs(np.linalg.norm(quad - edges, axis=2) - 0.02)
    end_edge = (width_error[:, 1::2].sum(axis=1) < width_error[:, ::2].sum(axis=1)).astype(int)
    middle = (quad + edges) / 2
    painted = middle[np.arange(len(quad))[:, None], end_edge[:, None] + [0, 2]]
    return {"painted": painted, "line_width_m": 0.02}


def ground_truth(meta):
    decoys = segments(d["xy"] for d in meta.get("case", {}).get("decoys", []))
    route = segments([meta.get("centerline", [])])
    paint = segments(meta.get("painted", [meta.get("centerline", [])]))
    marks = segments(meta.get("markings", []))
    # Exact duplicates (including reversed marking/decoy pairs) need only one segment.
    unique = {tuple(sorted((tuple(a), tuple(b)))): (a, b)
              for a, b in np.concatenate((paint, marks, decoys))}
    ink = np.asarray(list(unique.values()), float).reshape(-1, 2, 2)
    samples = []
    for a, b in ink:
        count = max(1, int(np.ceil(np.linalg.norm(b - a) / STEP - 1e-10)))
        samples.append(np.linspace(a, b, count + 1))
    if not samples:
        return ink, np.empty((0, 2)), np.empty(0, bool)
    points = np.concatenate(samples)
    _, keep = np.unique(np.rint(points / STEP).astype(np.int64), axis=0, return_index=True)
    points = points[keep]
    # A decoy can also appear inside painted or markings. Shared route points are main.
    is_decoy = ((segment_distance(points, decoys) <= 1e-7)
                & (segment_distance(points, route) > 1e-7))
    return ink, points, is_decoy


def fraction(hits, total):
    return {"hits": int(hits), "total": int(total), "rate": float(hits / total) if total else None}


def score_map(z, meta):
    ink, centers, is_decoy = ground_truth(meta)
    width = float(meta.get("line_width_m", 0.02))
    tolerance = width / 2 + 0.01
    xy = z["xy"].reshape(-1, 2)
    dist = z["dist"]
    track = z["track"]
    trajectory = track[:, 1:3] if len(track) else np.empty((0, 2))
    path = segments([trajectory])
    if len(trajectory) == 1:
        path = np.repeat(trajectory[:, None, :], 2, axis=1)
    proximity = segment_distance(centers, path)
    good = segment_distance(xy, ink) <= tolerance + 1e-10

    def coverage(cells):
        return cKDTree(cells).query(centers)[0] <= tolerance + 1e-10

    def recall(covered):
        result = {}
        for radius in RADII:
            view = proximity <= radius + 1e-10
            subsets = (view, view & ~is_decoy, view & is_decoy)
            result[str(radius)] = {name: fraction((covered & sel).sum(), sel.sum())
                                   for name, sel in zip(CLASSES, subsets)}
        return result

    covered = coverage(xy[good])
    scores = {"precision": fraction(good.sum(), len(xy)), "recall": recall(covered), "distance": {}}
    band_index = np.searchsorted([0.5, 0.9, 1.3], dist, side="right")
    for index, label in enumerate(BANDS):
        band = (band_index == index) & np.isfinite(dist) & (dist >= 0)
        accepted = band & good
        scores["distance"][label] = {"precision": fraction(accepted.sum(), band.sum()),
                                     "recall": recall(coverage(xy[accepted]))}
    if "conf" in z:
        conf = z["conf"]
        bin_index = np.searchsorted([0.2, 0.4, 0.6, 0.8], conf, side="right")
        calibration = {}
        for index in range(5):
            band = (bin_index == index) & np.isfinite(conf) & (conf >= 0) & (conf <= 1)
            calibration[f"{index / 5:.1f}-{(index + 1) / 5:.1f}"] = {
                **fraction((good & band).sum(), band.sum()), "confidence_sum": float(conf[band].sum()),
                "mean_confidence": float(conf[band].mean()) if band.any() else None}
        scores["calibration"] = calibration
    return scores, (ink, width, xy, good, centers[(proximity <= 1.0 + 1e-10) & ~covered])


def pool(scores):
    """Combine matching leaves by denominator, including optional confidence data."""
    if "hits" in scores[0]:
        merged = fraction(sum(s["hits"] for s in scores), sum(s["total"] for s in scores))
        if "confidence_sum" in scores[0]:
            count = merged["total"]
            confidence = sum(s["confidence_sum"] for s in scores)
            merged.update(confidence_sum=confidence, mean_confidence=confidence / count if count else None)
        return merged
    keys = dict.fromkeys(key for score in scores for key in score)
    return {key: pool([s[key] for s in scores if key in s]) for key in keys}


def overlay(path, data):
    ink, width, xy, good, missed = data
    fig, ax = plt.subplots(figsize=(7, 7), layout="constrained")
    lines = LineCollection(ink, colors="0.65", label="GT ink")
    ax.add_collection(lines)
    for points, color, label in ((xy[good], "forestgreen", "True positive"),
                                 (xy[~good], "firebrick", "False positive"),
                                 (missed, "royalblue", "Missed within 1 m")):
        ax.scatter(*points.T, s=3, c=color, label=label, zorder=3)
    ax.autoscale_view()
    ax.set(aspect="equal", xlabel="World x (m)", ylabel="World y (m)", title=path.stem)
    ax.legend(loc="best", markerscale=2)
    fig.canvas.draw()
    pixels_per_m = ax.transData.get_affine().get_matrix()[0, 0]
    lines.set_linewidth(width * pixels_per_m * 72 / fig.dpi)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def markdown(report):
    def fmt(metric):
        return (f"{metric['rate']:.1%} ({metric['hits']}/{metric['total']})"
                if metric["total"] else "—")

    rows = ["# Perception map evaluation", "",
            "| Run or aggregate | First distance (m) | Precision | Recall 1 m | Main | Decoy "
            "| Recall 0.5 m | Main | Decoy |", "|---|---|" + "---:|" * 7]
    items = [(r["run"], r["scores"]) for r in report["runs"]]
    if report["aggregate"]:
        items.append(("All runs", report["aggregate"]))
    items.extend((f"Profile {name}", score) for name, score in report["profiles"].items())
    for name, scores in items:
        for label, block in [("All", scores), *scores["distance"].items()]:
            metrics = [block["precision"], *(block["recall"][str(radius)][kind]
                        for radius in RADII for kind in CLASSES)]
            rows.append("| " + " | ".join([name.replace("|", "\\|"), label, *map(fmt, metrics)]) + " |")
    rows += ["", "Distance bands use the same recall denominator. Main includes non-decoy markings."]
    calibrated = [(name, score["calibration"]) for name, score in items if "calibration" in score]
    if calibrated:
        rows += ["", "| Run or aggregate | Confidence band | Mean confidence | Empirical precision |",
                 "|---|---|---:|---:|"]
        for name, bins in calibrated:
            name = name.replace("|", "\\|")
            for label, metric in bins.items():
                mean = f"{metric['mean_confidence']:.1%}" if metric["total"] else "—"
                rows.append(f"| {name} | {label} | {mean} | {fmt(metric)} |")
    if report["skipped"]:
        rows += ["", "## Skipped runs", "",
                 *(f"- {item['run']} — {item['reason']}" for item in report["skipped"])]
    return "\n".join(rows) + "\n"


def evaluate(runs_dir, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    report = {"runs": [], "skipped": [], "profiles": {}}
    profiles = {}
    for run in sorted(runs_dir.iterdir()):
        if not (run / "job.json").is_file():
            continue
        try:
            result = json.loads((run / "result.json").read_text())
            reason = str(result.get("reason", ""))
            if reason == "error" or reason.startswith("error:"):
                report["skipped"].append({"run": run.name, "reason": reason})
                continue
            job = json.loads((run / "job.json").read_text())
            world = Path(job["map"])
            if not world.exists() and world.parts[:2] == ("/", "ws"):
                world = ROOT.joinpath(*world.parts[2:])  # bench ran in the lab container
            sidecar = world.with_suffix(".json")
            meta = json.loads(sidecar.read_text()) if sidecar.is_file() else official_metadata(world)
            with np.load(run / "map_world.npz") as z:
                scores, data = score_map(z, meta)
        except (OSError, ValueError, KeyError) as error:
            report["skipped"].append({"run": run.name, "reason": str(error)})
            continue
        png = f"{run.name}.png"
        overlay(out_dir / png, data)
        profile = job.get("profile", "default")
        profiles.setdefault(profile, []).append(scores)
        report["runs"].append({"run": run.name, "profile": profile, "map": str(world),
                               "complete": result.get("complete"), "overlay": png, "scores": scores})
    report["aggregate"] = pool([r["scores"] for r in report["runs"]]) if report["runs"] else None
    report["profiles"] = {name: pool(scores) for name, scores in sorted(profiles.items())}
    (out_dir / "perception_map_eval.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    (out_dir / "perception_map_eval.md").write_text(markdown(report))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs_dir", type=Path)
    parser.add_argument("out_dir", type=Path)
    args = parser.parse_args()
    report = evaluate(args.runs_dir, args.out_dir)
    print(f"Scored {len(report['runs'])} runs, skipped {len(report['skipped'])}")
    return int(not report["runs"])


if __name__ == "__main__":
    raise SystemExit(main())
