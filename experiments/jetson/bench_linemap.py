#!/usr/bin/env python3
"""Measure LineMap.update wall time on a deterministic camera/pose replay."""
import argparse
import math
import os
from pathlib import Path
import subprocess
import sys
from time import perf_counter_ns

ROOT = Path(__file__).resolve().parents[2]


def replay(samples, warmup):
    import cv2
    import numpy as np

    sys.path.insert(0, str(ROOT / "src/line_follower"))
    from line_follower import line_map, vision

    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)
    frame_paths = [
        ROOT / "experiments/viewer/frame_640x480.png",
        ROOT / "experiments/results/lead/far15_video/03_01/video/cam_00057.jpg",
        ROOT / "experiments/results/lead/far15_video/03_01/video/cam_00131.jpg",
        ROOT / "experiments/worlds/suites/rand_sample/frames/rand_0005/03.png",
    ]
    frames = []
    for path in frame_paths:
        if path.is_file():
            gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if gray is not None and gray.shape == (vision.H, vision.W):
                frames.append(gray)

    observations = []
    for index in range(24):
        pitch = 0.05 * math.sin(index * 0.4)
        if frames:
            line, valid = vision.line_mask(frames[index % len(frames)], theta=pitch)
        else:
            rows, cols = np.indices((vision.BH, vision.BW))
            center = vision.BW / 2 + 22 * np.sin(rows / 44 + index * 0.15)
            _, valid = vision.bev_warp(np.zeros((vision.H, vision.W), np.uint8), pitch)
            line = ((abs(cols - center) <= 3) & (valid > 0)).astype(np.uint8) * 255
        far = valid.copy()
        far[int(0.3 * vision.BH):] = 0
        observations.append((line, valid, far))

    print(f"X_FAR={vision.X_FAR:g} patch={line_map.PATCH} "
          f"saved_frames={len(frames)} OpenCV={cv2.__version__} threads=1", flush=True)
    for use_far in (False, True):
        mapping = line_map.LineMap()
        times = []
        for index in range(samples + warmup):
            angle = index / 45
            pose = (1.8 * math.cos(angle), 1.8 * math.sin(angle), angle + math.pi / 2)
            line, valid, far = observations[index % len(observations)]
            band = far if use_far else None
            mapping.freeze(*pose[:2], radius=0.10)
            start = perf_counter_ns()
            mapping.update(line, valid, pose, add_only=band, far_mask=band)
            elapsed = perf_counter_ns() - start
            if index >= warmup:
                times.append(elapsed / 1e6)
        p50, p95 = np.percentile(times, [50, 95])
        print(f"  far_mask={use_far!s:5} n={samples} p50={p50:.3f} ms p95={p95:.3f} ms",
              flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=600)
    parser.add_argument("--warmup", type=int, default=60)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.samples < 1 or args.warmup < 0:
        parser.error("samples must be positive and warmup must be nonnegative")
    if args.worker:
        replay(args.samples, args.warmup)
        return
    for x_far in ("0.85", "1.3"):
        subprocess.run(
            [sys.executable, __file__, "--worker", "--samples", str(args.samples),
             "--warmup", str(args.warmup)],
            env={**os.environ, "LF_X_FAR": x_far},
            check=True,
        )


if __name__ == "__main__":
    main()
