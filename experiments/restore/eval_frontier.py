#!/usr/bin/env python3
"""Score ONNX restoration on validation patches using the robot's ridge walker."""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src/line_follower"))
from line_follower.restore import PRES, X1, Y0, Restorer

LENGTH = 0.4
THRESHOLD = 0.6
TOLERANCE = 0.02
FRONTIER_KERNEL = np.ones((5, 5), dtype=np.uint8)


def frontier_seeds(line, seen):
    adjacent = cv2.dilate((seen == 0).astype(np.uint8), FRONTIER_KERNEL)
    count, labels = cv2.connectedComponents((line > 0).astype(np.uint8) & adjacent, connectivity=8)
    line_rows, line_cols = np.nonzero(line)
    line_x, line_y = Restorer.to_robot(line_cols, line_rows)
    for label in range(1, count):
        rows, cols = np.nonzero(labels == label)
        row, col = rows[0], cols[0]  # first row-major pixel at maximum forward x
        x, y = Restorer.to_robot(col, row)
        if x <= 0.2:
            continue
        distance = np.hypot(line_x - x, line_y - y)
        near = np.flatnonzero(distance < 0.12)
        order = near[np.argsort(-distance[near])]
        yield (row, col), (x, y), list(zip(line_x[order], line_y[order]))


def distance_to(mask):
    if not mask.any():
        return np.full(mask.shape, np.inf, dtype=np.float32)
    return cv2.distanceTransform((mask == 0).astype(np.uint8), cv2.DIST_L2,
                                 cv2.DIST_MASK_PRECISE) * PRES


def score_sample(walker, patch, route, unseen_route, prob):
    line, seen = patch
    seeds = np.zeros(line.shape, dtype=np.uint8)
    predicted = np.zeros_like(seeds)
    for (row, col), start, known in frontier_seeds(line, seen):
        seeds[row, col] = 1
        for x, y in walker.extend_ridge(prob, start, known, LENGTH, THRESHOLD):
            r = int(round((X1 - x) / PRES - 0.5))
            c = int(round((Y0 - y) / PRES - 0.5))
            if not seen[r, c]:
                predicted[r, c] = 1
    target = (unseen_route > 0) & (distance_to(seeds) <= LENGTH + 1e-7)
    correct = (predicted > 0) & (distance_to(route) <= TOLERANCE)
    covered = target & (distance_to(predicted) <= TOLERANCE)
    return correct.sum(), np.count_nonzero(predicted), covered.sum(), target.sum()


def logits_batches(model, patches, batch):
    import onnxruntime as ort

    options = ort.SessionOptions()
    options.intra_op_num_threads = 4
    session = ort.InferenceSession(str(model), options, providers=["CPUExecutionProvider"])
    inp = session.get_inputs()[0]
    if isinstance(inp.shape[0], int):
        batch = inp.shape[0]
    for i in range(0, len(patches), batch):
        x = patches[i:i + batch].astype(np.float32)
        yield session.run(None, {inp.name: x})[0][:, 0]


def latency(model, patches):
    net = cv2.dnn.readNetFromONNX(str(model))
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    timings = []
    for patch in patches[:110].astype(np.float32):
        start = time.perf_counter()
        net.setInput(patch[None])
        net.forward()
        timings.append(1000 * (time.perf_counter() - start))
    measured = timings[10:]
    return dict(opencv=cv2.__version__, threads=cv2.getNumThreads(), samples=len(measured),
                median_ms=float(np.median(measured)), p95_ms=float(np.percentile(measured, 95)))


def evaluate(model, data, batch):
    walker = Restorer.__new__(Restorer)
    totals = np.zeros(4, dtype=np.int64)
    offset = 0
    for logits in logits_batches(model, data["x"], batch):
        probs = 1 / (1 + np.exp(-np.clip(logits, -80, 80)))
        for i, prob in enumerate(probs, start=offset):
            totals += score_sample(walker, data["x"][i], data["y"][i], data["unseen"][i], prob)
        offset += len(probs)
    correct, predicted, covered, target = map(int, totals)
    return dict(samples=offset, correct_points=correct, predicted_points=predicted,
                covered_route_pixels=covered, target_route_pixels=target,
                chain_precision=correct / predicted if predicted else None,
                chain_recall=covered / target if target else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--val", type=Path, required=True)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--latency", action="store_true", help="benchmark OpenCV DNN only")
    args = parser.parse_args()
    cv2.setNumThreads(4)
    with np.load(args.val) as archive:
        data = dict(archive)
    result = latency(args.model, data["x"]) if args.latency else evaluate(args.model, data, args.batch)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
