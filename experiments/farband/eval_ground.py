"""Evaluate the flat-ground far-band detectors and regenerate REPORT_ground_error.md."""

import json
import os
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
os.environ["LF_X_FAR"] = "1.6"
sys.path.insert(0, str(ROOT / "src/line_follower"))
from line_follower import vision
from capture import dense, ground_grid

MODELS = ("farband_w8h2", "farband_w8h2_hill")
DETECTORS = (*MODELS, "threshold")
EDGES_CM = (85, 95, 105, 115, 125, 135, 145, 155, 160)
BINS = len(EDGES_CM) - 1
DUMMY = np.zeros((vision.H, vision.W), np.uint8)


def world_points(robot, pose):
    x, y, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    return robot @ np.array([[c, s], [-s, c]]) + [x, y]


def project(strip):
    image = DUMMY.copy()
    image[256:320] = strip.astype(np.uint8) * 255
    line, valid = vision.far_band_mask(DUMMY, 0.0, 0.85, 1.6, line_img=image)
    return line > 0, valid > 0


def reference(path, labels, poses, grid):
    """Verify sidecar support in image space using the original 2x2 subpixel grid."""
    if not path.is_file():
        return None, 0, None, None
    meta = json.loads(path.read_text())
    polylines = [meta["centerline"], *meta["markings"]]
    tree = cKDTree(np.vstack([dense(line) for line in polylines if len(line) > 1]))
    intersection = union = 0
    informative = 0
    min_iou = 1.0
    for i in np.linspace(0, len(labels) - 1, min(12, len(labels)), dtype=int):
        ground = world_points(grid, poses[i])
        d = tree.query(ground, distance_upper_bound=0.01)[0]
        reprojection = np.isfinite(d).reshape(64, 2, 640, 2).any((1, 3))
        stored = labels[i] > 0
        sample_union = np.count_nonzero(reprojection | stored)
        if sample_union:
            sample_intersection = np.count_nonzero(reprojection & stored)
            intersection += sample_intersection
            union += sample_union
            informative += 1
            min_iou = min(min_iou, sample_intersection / sample_union)
    iou = intersection / union if union else None
    matched = informative >= 3 and iou >= 0.90 and min_iou >= 0.75
    return tree if matched else None, informative, iou, min_iou if informative else None


def predict(strip, nets):
    rgb = cv2.resize(strip, (320, 64), interpolation=cv2.INTER_AREA)
    blob = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255
    for name, net in nets.items():
        net.setInput(blob, "rgb")
        logits = net.forward("logits")
        if logits.shape != (1, 1, 64, 320):
            raise ValueError(f"Unexpected {name} output shape {logits.shape}")
        positive = (logits[0, 0] > 0).astype(np.uint8)
        yield name, cv2.resize(positive, (640, 64), interpolation=cv2.INTER_NEAREST)
    gray = cv2.cvtColor(strip, cv2.COLOR_RGB2GRAY)
    yield "threshold", gray < min(125, 0.62 * float(np.median(gray)))


def nearest(points, targets):
    if len(points) and len(targets):
        return cKDTree(targets).query(points)[0]
    return np.full(len(points), np.inf)


def quantile(values, q):
    """Inverse empirical CDF, including predictions with no available reference."""
    index = int(np.ceil(q * len(values))) - 1
    return np.partition(values, index)[index]


def summarize(chunks, hits, true_counts):
    rows = []
    for parts, recalled, total in zip(chunks, hits, true_counts):
        d = np.concatenate(parts)
        if len(d):
            stats = (np.mean(d <= 0.02 + 1e-12), np.mean(d <= 0.03 + 1e-12),
                     100 * quantile(d, 0.50), 100 * quantile(d, 0.90))
        else:
            stats = (None,) * 4
        rows.append((*stats, recalled / total if total else None, len(d), total))
    return rows


def table(rows):
    yield "| Forward bin (m) | P≤2 cm | P≤3 cm | Median (cm) | P90 (cm) | R≤3 cm | Pred cells | True cells |"
    yield "|---|---:|---:|---:|---:|---:|---:|---:|"
    formats = (".1%", ".1%", ".2f", ".2f", ".1%", ",d", ",d")
    for lo, hi, row in zip(EDGES_CM, EDGES_CM[1:], rows):
        values = ["—" if v is None else format(v, fmt) for v, fmt in zip(row, formats)]
        yield "| " + " | ".join([f"{lo/100:.2f}–{hi/100:.2f}", *values]) + " |"


def qualifying_end(rows):
    end = EDGES_CM[0]
    for upper, (_, p3, median, *_) in zip(EDGES_CM[1:], rows):
        if p3 is None or p3 < 0.70 or median > 2.0:
            break
        end = upper
    return end


def main():
    cv2.setNumThreads(4)
    nets = {}
    for name in MODELS:
        net = cv2.dnn.readNetFromONNX(str(HERE / "models" / f"{name}.onnx"))
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        nets[name] = net

    paths = sorted((HERE / "data").glob("*.npz"))[-10:]
    if len(paths) != 10:
        raise ValueError("Expected ten held-out maps")
    sidecars = ROOT / "experiments/worlds/suites/tiles60/maps"
    grid = ground_grid().reshape(-1, 2)
    _, valid = project(np.zeros((64, 640), bool))
    vr, vc = np.where(valid)
    gx, gy = vision.bev_to_robot(vc, vr)
    ground = np.column_stack((gx, gy))
    # Integer millimetres avoid assigning exact decimal edges to the preceding bin.
    bin_ids = np.searchsorted(np.array(EDGES_CM[1:-1]) * 10, np.rint(gx * 1000), side="right")
    variants = [(name, thin) for name in DETECTORS for thin in (False, True)]
    errors = {key: [[] for _ in range(BINS)] for key in variants}
    hits = {key: np.zeros(BINS, dtype=np.int64) for key in variants}
    true_counts = np.zeros(BINS, dtype=np.int64)
    audit = []
    n_frames = 0

    for path in paths:
        with np.load(path) as data:
            strips, labels, poses = data["x"], data["y"], data["pose"]
            if not np.array_equal(data["rows"], [256, 320]):
                raise ValueError(f"Unexpected strip rows in {path}")
        if not len(strips):
            audit.append((path.stem, 0, 0, None, None, "Empty capture"))
            print(f"{path.stem}: empty capture", flush=True)
            continue
        if (strips.shape[1:] != (64, 640, 3) or labels.shape != strips.shape[:3]
                or poses.shape != (len(strips), 3)):
            raise ValueError(f"Unexpected array shapes in {path}")
        tree, n, iou, worst = reference(sidecars / f"{path.stem}.json", labels, poses, grid)
        source = "Sidecar" if tree is not None else "Projected labels"
        audit.append((path.stem, len(strips), n, iou, worst, source))
        print(f"{path.stem}: {len(strips)} frames, {source}, support IoU {iou}, min {worst}", flush=True)
        n_frames += len(strips)
        for strip, label, pose in zip(strips, labels, poses):
            world = world_points(ground, pose)
            if tree is None:
                gt_mask = project(label > 0)[0][vr, vc]
            else:
                gt_mask = tree.query(world, distance_upper_bound=0.01)[0] <= 0.01
            truth = world[gt_mask]
            gt_bins = bin_ids[gt_mask]
            true_counts += np.bincount(gt_bins, minlength=BINS)

            for name, strip_mask in predict(strip, nets):
                raw = project(strip_mask)[0]
                thinned = vision.thin_across(raw, max_run=16, keep=1)
                for thin, mask in ((False, raw), (True, thinned)):
                    key = name, thin
                    selected = mask[vr, vc]
                    predicted = world[selected]
                    d = tree.query(predicted)[0] if tree is not None else nearest(predicted, truth)
                    pred_bins = bin_ids[selected]
                    for b in range(BINS):
                        errors[key][b].append(d[pred_bins == b])
                    detected = nearest(truth, predicted) <= 0.03 + 1e-12
                    hits[key] += np.bincount(gt_bins[detected], minlength=BINS)

    assert n_frames and true_counts.sum()
    results = {key: summarize(errors[key], hits[key], true_counts) for key in variants}
    for name in DETECTORS:
        raw, thin = (results[name, mode] for mode in (False, True))
        assert all(t[5] <= r[5] and t[6] == r[6] and t[4] <= r[4] for r, t in zip(raw, thin))
    write_report(results, audit, n_frames, gx)


def write_report(results, audit, n_frames, gx):
    ends = {key: qualifying_end(rows) for key, rows in results.items()}
    learned = [(name, thin) for name in MODELS for thin in (False, True)]
    best = max(learned, key=lambda key: (
        ends[key],
        min((row[1] for upper, row in zip(EDGES_CM[1:], results[key]) if upper <= ends[key]),
            default=0.0),
    ))
    end = ends[best]
    recommendation = (
        f"Use a **{end/100:.2f} m far-band end** on the evaluated flat ground. "
        f"`{best[0]}` ({'thinned' if best[1] else 'raw'}) meets both criteria "
        f"in every bin up to that cut."
        if end > EDGES_CM[0] else
        "The learned models do not support extending the far band from 0.85 m under the specified criteria."
    )
    lines = [
        "# Far-band ground placement error", "", recommendation, "",
        "The gate uses unrounded median ≤2 cm and P≤3 cm ≥70% in every successive bin from 0.85 m. "
        "Among learned variants with equal reach, the recommendation favors the highest worst-bin "
        "P≤3 cm within that reach. Recall is reported separately.", "",
        "| Detector | Raw end (m) | Thinned end (m) |", "|---|---:|---:|",
    ]
    for name in DETECTORS:
        lines.append(f"| {name} | {ends[name, False]/100:.2f} | {ends[name, True]/100:.2f} |")
    lines += [
        "", "## Method", "",
        f"- Evaluated all {n_frames:,} stored flat-ground frames from the last ten sorted map names. "
        "The empty capture contributed zero frames. This evaluation covers flat ground at zero pitch.",
        "- Both ONNX models ran on the OpenCV CPU backend. RGB columns were downscaled with area "
        "interpolation and normalized by 255. Logits >0 were upscaled by nearest neighbor. "
        "The threshold baseline used RGB-to-gray and gray < min(125, 0.62 × strip median). "
        "The full strip was the available proxy for bottom-row ground brightness and can include non-ground pixels.",
        "- All masks used the 640×480 image layout with the strip in rows 256–319 and "
        "`far_band_mask(..., 0.0, 0.85, 1.6, line_img=...)`. Predicted 5 mm BEV cells "
        "were converted through `bev_to_robot` and the stored pose into world XY. "
        f"The valid grid spans {gx.min():.3f}–{gx.max():.3f} m after border erosion. "
        "Bins include their lower edge and exclude their upper edge, with the final bin closed at 1.60 m.",
        "- Sidecar checks reprojected the centerline and markings using the capture's 2×2 "
        "subpixel grid and 1 cm half-width at up to 12 evenly spaced poses per map. "
        "A sidecar required at least three informative samples (nonempty union), pooled "
        "positive-support IoU ≥0.90 and minimum informative-frame IoU ≥0.75. "
        "Unverified maps used the stored positive-coverage pixels (y > 0), binarized before the same BEV warp.",
        "- For verified sidecars, forward error is measured to the closest centerline or marking "
        "sample on the world polyline (spacing ≤2.5 mm). Recall targets valid cells within 1 cm "
        "of that geometry. For label fallback, forward error and recall use the projected positive-coverage "
        "cells as reference, so errors there measure distance to rasterized line support.",
        "- Distances are Euclidean within a frame. Nearest neighbors may cross bin boundaries. "
        "Precision bins by predicted x, recall by true x. Cell counts and distances are pooled "
        "across frames. A prediction with no label reference in the band has infinite distance. "
        "A missing prediction contributes zero recall hits. Quantiles use the inverse empirical CDF. "
        "Thinning uses `thin_across(mask, max_run=16, keep=1)` on each boolean BEV prediction.",
        "", "## Sidecar audit", "",
        "| Map | Frames | Informative checks | Pooled IoU | Min IoU | Ground truth |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for name, frames, n, iou, worst, source in audit:
        pooled = "—" if iou is None else f"{iou:.3f}"
        minimum = "—" if worst is None else f"{worst:.3f}"
        lines.append(f"| {name} | {frames} | {n} | {pooled} | {minimum} | {source} |")
    lines += ["", "## Results", ""]
    for (name, thin), rows in results.items():
        lines += [f"### {name} ({'thinned' if thin else 'raw'})", "", *table(rows), ""]
    (HERE / "REPORT_ground_error.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote report from {n_frames} frames. {recommendation}", flush=True)


if __name__ == "__main__":
    main()
