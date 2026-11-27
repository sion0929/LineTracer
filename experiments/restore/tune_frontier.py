#!/usr/bin/env python3
"""Offline frontier sweep using the production ridge walker and cached ONNX probabilities."""

import argparse
import hashlib
import math
from pathlib import Path

import cv2
import numpy as np

from eval_frontier import FRONTIER_KERNEL, PRES, X1, Y0, Restorer, distance_to, logits_batches

THRESHOLDS = (0.6, 0.7, 0.8, 0.9)
LENGTHS = (0.1, 0.2, 0.3, 0.4)
INTERMEDIATE_LENGTHS = (0.12, 0.14, 0.16, 0.18)
FIXED_REACH = 0.4
GATES = (0.0, 0.75, 0.85, 0.9, 0.95, 0.98, 0.99)
RULES = ("forward", "direction")
FIELDS = ("correct", "predicted", "covered", "target", "covered_fixed",
          "target_fixed", "wrong", "active", "any_bad")


def digest(path):
    with path.open("rb") as handle:
        return hashlib.file_digest(handle, "sha256").hexdigest()


def cache(model, val, patches, path):
    identifiers = (digest(model), digest(val))
    if path.exists():
        with np.load(path) as archive:
            assert tuple(archive["sha256"]) == identifiers, "Cache provenance mismatch"
            return archive["prob"]

    probabilities = np.empty((len(patches), *patches.shape[-2:]), np.float32)
    offset = 0
    for logits in logits_batches(model, patches, 16):
        probabilities[offset:offset + len(logits)] = 1 / (1 + np.exp(-np.clip(logits, -80, 80)))
        offset += len(logits)
    assert offset == len(patches)
    temporary = path.with_suffix(".part.npz")
    np.savez(temporary, prob=probabilities, sha256=identifiers)
    temporary.replace(path)
    return probabilities


def seed_pairs(line, seen):
    adjacent = cv2.dilate((seen == 0).astype(np.uint8), FRONTIER_KERNEL)
    count, labels = cv2.connectedComponents((line > 0).astype(np.uint8) & adjacent, connectivity=8)
    line_rc = np.column_stack(np.nonzero(line))
    line_x, line_y = Restorer.to_robot(line_rc[:, 1], line_rc[:, 0])
    interior = (seen > 0) & (adjacent == 0)
    interior_rc = line_rc[interior[tuple(line_rc.T)]]
    for label in range(1, count):
        pixels = np.column_stack(np.nonzero(labels == label))
        pixels = pixels[X1 - (pixels[:, 0] + 0.5) * PRES > 0.2]
        if not len(pixels):
            continue
        center = pixels.mean(axis=0)
        offsets = center - interior_rc
        local = offsets[np.sum(offsets ** 2, axis=1) < (0.12 / PRES) ** 2]
        direction = pixels[0]
        if len(local):
            outward = local.mean(axis=0)
            if np.linalg.norm(outward) > 0:
                direction = pixels[np.argmax((pixels - center) @ outward)]
        pair = []
        for row, col in (pixels[0], direction):
            x, y = Restorer.to_robot(col, row)
            distances = np.hypot(line_x - x, line_y - y)
            near = np.flatnonzero(distances < 0.12)
            order = near[np.argsort(-distances[near])]
            pair.append(((row, col), (x, y), list(zip(line_x[order], line_y[order]))))
        yield pair


def walk(walker, prob, start, known, threshold):
    chain = walker.extend_ridge(prob, start, known, FIXED_REACH, threshold)
    cumulative = []
    steps = []
    last = start
    length = 0.0
    for x, y in chain:
        step = math.hypot(x - last[0], y - last[1])
        length += step
        steps.append(step)
        cumulative.append(length)
        last = (x, y)
    xy = np.asarray(chain).reshape(-1, 2)
    rows = np.rint((X1 - xy[:, 0]) / PRES - 0.5).astype(int)
    cols = np.rint((Y0 - xy[:, 1]) / PRES - 0.5).astype(int)
    return np.asarray(cumulative), np.asarray(steps), rows, cols


def score_patch(walker, patch, route, unseen, prob, lengths=LENGTHS):
    line, seen = patch
    pairs = list(seed_pairs(line, seen))
    if not pairs:
        return None
    route_dist = distance_to(route)
    target_rc = np.column_stack(np.nonzero(unseen))
    fixed_seeds = np.asarray([pair[0][0] for pair in pairs])
    fixed_d2 = ((target_rc[:, None] - fixed_seeds) ** 2).sum(axis=2).min(axis=1)
    fixed_target = fixed_d2 <= ((FIXED_REACH + 1e-7) / PRES) ** 2
    scores = np.zeros((len(RULES), len(THRESHOLDS), len(lengths), len(GATES), len(FIELDS)),
                      dtype=np.int64)
    memo = {}
    for si, seeds in enumerate(zip(*pairs)):
        seed_rc = np.asarray([seed[0] for seed in seeds])
        seed_d2 = ((target_rc[:, None] - seed_rc) ** 2).sum(axis=2).min(axis=1)
        for ti, threshold in enumerate(THRESHOLDS):
            walks = []
            for rc, start, known in seeds:
                key = (rc, threshold)
                if key not in memo:
                    memo[key] = walk(walker, prob, start, known, threshold)
                walks.append(memo[key])
            for li, length in enumerate(lengths):
                target = seed_d2 <= ((length + 1e-7) / PRES) ** 2
                preds = [set() for _ in GATES]
                wrong = np.zeros(len(GATES), dtype=bool)
                any_bad = np.zeros(len(GATES), dtype=bool)
                for cumulative, steps, rows, cols in walks:
                    end = np.searchsorted(cumulative, length, side="right")
                    if not end:
                        continue
                    rows, cols = rows[:end], cols[:end]
                    confidence = float(prob[rows, cols].mean())
                    unseen_points = seen[rows, cols] == 0
                    bad_steps = unseen_points & (route_dist[rows, cols] > 0.03)
                    is_wrong = steps[:end][bad_steps].sum() >= 0.05 - 1e-12
                    pixels = list(zip(rows[unseen_points], cols[unseen_points]))
                    for gi, gate in enumerate(GATES):
                        if confidence < gate:
                            break
                        preds[gi].update(pixels)
                        wrong[gi] |= is_wrong
                        any_bad[gi] |= bad_steps.any()
                scored = {}
                for gi, pred in enumerate(preds):
                    key = frozenset(pred)
                    if key not in scored:
                        correct = covered = covered_fixed = 0
                        if pred:
                            rows, cols = np.asarray(sorted(pred)).T
                            correct = np.count_nonzero(route_dist[rows, cols] <= 0.02)
                            # At 1.25 cm/pixel, a 2 cm disk contains exactly the 3x3 neighborhood.
                            covered_mask = ((abs(target_rc[:, None] - np.column_stack((rows, cols)))
                                             <= 1).all(axis=2)).any(axis=1)
                            covered = np.count_nonzero(covered_mask & target)
                            covered_fixed = np.count_nonzero(covered_mask & fixed_target)
                        scored[key] = (correct, covered, covered_fixed)
                    correct, covered, covered_fixed = scored[key]
                    scores[si, ti, li, gi] = (
                        correct, len(pred), covered, target.sum(), covered_fixed, fixed_target.sum(),
                        wrong[gi], bool(pred), any_bad[gi])
    return scores


def evaluate(data, probabilities, limit, lengths):
    walker = Restorer.__new__(Restorer)
    totals = np.zeros((len(RULES), len(THRESHOLDS), len(lengths), len(GATES), len(FIELDS)),
                      dtype=np.int64)
    for index, args in enumerate(zip(data["x"], data["y"], data["unseen"], probabilities), 1):
        result = score_patch(walker, *args, lengths=lengths)
        if result is not None:
            totals += result
        if index % 1000 == 0:
            print(f"Scored {index} patches", flush=True)
        if index == limit:
            break
    return totals, index


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--val", type=Path, default=Path("/home/rvlab/restore_data/val.npz"))
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--cache", type=Path, required=True)
    parser.add_argument("--scores", type=Path, required=True)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--intermediate-lengths", action="store_true",
                        help="sweep 0.12, 0.14, 0.16 and 0.18 m using the same cache")
    args = parser.parse_args()
    cv2.setNumThreads(4)
    with np.load(args.val) as archive:
        data = dict(archive)
    probabilities = cache(args.model, args.val, data["x"], args.cache)
    assert probabilities.shape == data["y"].shape
    lengths = INTERMEDIATE_LENGTHS if args.intermediate_lengths else LENGTHS
    totals, samples = evaluate(data, probabilities, args.limit, lengths)
    np.savez(args.scores, totals=totals, samples=samples, rules=RULES,
             thresholds=THRESHOLDS, lengths=lengths, gates=GATES, fields=FIELDS)


if __name__ == "__main__":
    main()
