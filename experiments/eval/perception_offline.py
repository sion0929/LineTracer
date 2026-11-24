#!/usr/bin/env python3
"""Offline comparison of line-perception candidates on labelled camera frames (dev tool).

Every candidate maps a 480x640 BGR camera frame to a boolean line mask in IMAGE space.
Ground truth: experiments/dl dataset masks (0 background, 1 line, 2 obstacle), produced by
projecting the true line / box geometry. Frames whose robot z is above the flat value are
reported separately as "on a hill".

  python experiments/eval/perception_offline.py --data DATASET_TEST_DIR --models DIR --out out.json

Metrics per candidate and map: line IoU, precision, recall, obstacle pixels wrongly called
line (dark boxes), and CPU latency (1 thread and 4 threads). CPU only on purpose: the external
grader may have no GPU; the Orin Nano path is TensorRT later.
"""
import argparse
import glob
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/line_follower"))


# ---------------------------------------------------------------- candidates
def fixed_gray(bgr, thr=80):
    """Original experiments/follower.py rule: gray < 80."""
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) < thr


def median_ratio(bgr, ratio=0.62, cap=125.0):
    """User package rule (vision.line_mask) applied in image space: darker than 0.62 x median."""
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    lower = g[g.shape[0] // 2:]                      # ground half of the image
    t = min(cap, ratio * float(np.median(lower)))
    m = g < t
    m[: g.shape[0] // 2 - 20] = False                # above the horizon is not ground
    return m


def otsu(bgr):
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    roi = g[g.shape[0] // 2:]
    t, _ = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    m = g < t
    m[: g.shape[0] // 2 - 20] = False
    return m


def adaptive_gauss(bgr, block=51, c=15):
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    m = cv2.adaptiveThreshold(g, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, block, c) > 0
    m[: g.shape[0] // 2 - 20] = False
    return m


class RobotisHSV:
    """ROBOTIS turtlebot3_autorace detect_lane (Apache-2.0) colour mask, adapted to a black line:
    HSV inRange with an upper value bound that is nudged by 5 until the detected pixel count is
    inside a band (their 5000-35000 px on a 1000x600 BEV ~ 0.8-5.8 %). Frames here are
    independent, so the adaptation is iterated to convergence per frame."""

    def __init__(self, v_high=80, s_high=255, lo=0.004, hi=0.05):
        self.v0, self.s_high, self.lo, self.hi = v_high, s_high, lo, hi

    def __call__(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        hsv[: hsv.shape[0] // 2 - 20] = 255          # ignore above the horizon
        v = self.v0
        area = hsv.shape[0] * hsv.shape[1]
        for _ in range(30):
            m = cv2.inRange(hsv, (0, 0, 0), (179, self.s_high, v)) > 0
            frac = m.sum() / area
            if frac > self.hi and v > 10:
                v -= 5
            elif frac < self.lo and v < 200:
                v += 5
            else:
                break
        return m


class Onnx:
    def __init__(self, path, threads):
        import onnxruntime as ort
        o = ort.SessionOptions()
        o.intra_op_num_threads, o.inter_op_num_threads = threads, 1
        self.s = ort.InferenceSession(str(path), o, providers=["CPUExecutionProvider"])
        self.inp = self.s.get_inputs()[0]
        sys.path.insert(0, str(Path(path).resolve().parents[2]))
        from experiments.dl.infer import prepare, decode   # same pre/post-processing as training
        self.prepare, self.decode = prepare, decode

    def __call__(self, bgr):
        h, w = self.inp.shape[2], self.inp.shape[3]     # v1 is 240x320, v2/v3 full 480x640
        scores, = self.s.run(None, {self.inp.name: self.prepare(bgr, (w, h))})
        line, _ = self.decode(scores)
        return line


# ---------------------------------------------------------------- evaluation
def load_frames(test_dir):
    for map_dir in sorted(Path(test_dir).iterdir()):
        meta = {}
        fj = map_dir / "frames.jsonl"
        if fj.is_file():
            for line in fj.read_text().splitlines():
                d = json.loads(line)
                meta[d["file"]] = d
        for img in sorted((map_dir / "image").glob("*.png")):
            yield map_dir.name, img, map_dir / "mask" / img.name, meta.get(img.name, {})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="dataset test dir with <map>/{image,mask,frames.jsonl}")
    ap.add_argument("--models", default="", help="dir with line_seg*.onnx (optional)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--limit", type=int, default=0, help="frames per map (0 = all)")
    ap.add_argument("--hill-z", type=float, default=0.015, help="robot z above this = on a hill")
    a = ap.parse_args()
    cv2.setNumThreads(1)

    cands = {
        "fixed_gray80": fixed_gray,
        "user_median0.62": median_ratio,
        "otsu": otsu,
        "adaptive_gauss": adaptive_gauss,
        "robotis_hsv_adapt": RobotisHSV(),
    }
    onnx_paths = sorted(glob.glob(os.path.join(a.models, "line_seg*.onnx"))) if a.models else []
    for p in onnx_paths:
        name = Path(p).stem
        cands[f"{name}@1t"] = Onnx(p, 1)
        cands[f"{name}@4t"] = Onnx(p, 4)

    frames = list(load_frames(a.data))
    if a.limit:
        keep, seen = [], {}
        for f in frames:
            seen[f[0]] = seen.get(f[0], 0) + 1
            if seen[f[0]] <= a.limit:
                keep.append(f)
        frames = keep
    imgs = [(m, cv2.imread(str(i)), cv2.imread(str(k), cv2.IMREAD_UNCHANGED), meta) for m, i, k, meta in frames]
    print(f"{len(imgs)} frames from {sorted({m for m, *_ in imgs})}")

    results = {}
    for name, fn in cands.items():
        acc = {}
        times = []
        for map_name, bgr, gt, meta in imgs:
            if gt.ndim == 3:
                gt = gt[..., 0]
            t0 = time.perf_counter()
            pred = fn(bgr)
            times.append(time.perf_counter() - t0)
            line, obst = gt == 1, gt == 2
            z = (meta.get("position") or [0, 0, 0])[2]
            for key in (map_name, "ALL", "hill" if z > a.hill_z else "flat"):
                s = acc.setdefault(key, np.zeros(5))
                s += [(pred & line).sum(), (pred | line).sum(), pred.sum(), line.sum(), (pred & obst).sum()]
                acc.setdefault(key + "#obst", np.zeros(1))[0] += obst.sum()
        res = {"ms_mean": round(1000 * float(np.mean(times)), 2), "ms_p95": round(1000 * float(np.percentile(times, 95)), 2)}
        for key, s in acc.items():
            if key.endswith("#obst"):
                continue
            inter, union, npred, ngt, onl = s
            res[key] = dict(iou=round(inter / max(union, 1), 4), precision=round(inter / max(npred, 1), 4),
                            recall=round(inter / max(ngt, 1), 4),
                            obstacle_as_line=round(onl / max(acc[key + "#obst"][0], 1), 4))
        results[name] = res
        print(f"{name:26} IoU all {res['ALL']['iou']:.3f}  official {res.get('official', {}).get('iou', float('nan')):.3f}  "
              f"hill {res.get('hill', {}).get('iou', float('nan')):.3f}  flat {res.get('flat', {}).get('iou', float('nan')):.3f}  "
              f"box->line {res['ALL']['obstacle_as_line']:.3f}  {res['ms_mean']:.1f} ms")
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(dict(frames=len(imgs), results=results), indent=1))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
