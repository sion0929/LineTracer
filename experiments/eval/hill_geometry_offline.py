#!/usr/bin/env python3
"""Offline check: can the known line width (2 cm) fix flat-ground IPM on hills? (dev tool)

For sampled points of the TRUE 3D centre line that the camera sees (0.31-0.85 m ahead) we
compare two runtime-feasible 3D estimates against the truth:

  ipm    ray through the pixel intersected with the robot's own ground plane (current method;
         camera pose from the body attitude = what the IMU gives)
  width  depth from the line's horizontal pixel width: Z = f * 0.02 / (w_px * cos(phi)),
         phi = line direction vs. forward (taken from the IPM estimate), then the 3D point
         along the pixel ray. Gives height too.

Widths are measured on the GT mask (upper bound) and on a segmentation model's mask
(--model, realistic). Points are split by how far the true ground height at the point
differs from the robot's own ground level (flat / hill).

  python experiments/eval/hill_geometry_offline.py --dataset .../dataset-v1 --out out.json [--model m.onnx]
"""
import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

LINE_W = 0.020
WIDTH, HEIGHT = 640, 480
F = WIDTH / (2 * math.tan(1.085595 / 2))
CX, CY = (WIDTH - 1) / 2, (HEIGHT - 1) / 2
CAM = np.array([0.076, 0.0, 0.093])           # camera in the body frame (as the dataset renderer)
BODY_Z = 0.009                                 # body origin above its own ground


def qmat(x, y, z, w):
    return np.array([[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
                     [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
                     [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])


def dense(poly, step=0.04):
    poly = np.asarray(poly, float)
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b[:2] - a[:2]) / step)))
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
    return np.array(out)


def run_width(mask, u, v, max_w=80):
    """Horizontal run length of mask pixels through (u, v) (sub-pixel ends not attempted)."""
    if not (0 <= v < HEIGHT and 0 <= u < WIDTH) or not mask[v, u]:
        # allow 1-2 px misalignment of the projected centre
        for du in (-1, 1, -2, 2):
            if 0 <= u + du < WIDTH and mask[v, u + du]:
                u += du
                break
        else:
            return None
    row = mask[v]
    a = u
    while a > 0 and row[a - 1] and u - a < max_w:
        a -= 1
    b = u
    while b < WIDTH - 1 and row[b + 1] and b - u < max_w:
        b += 1
    return b - a + 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True, help="dataset-v1 dir (train/val/test + worlds)")
    ap.add_argument("--splits", default="train,val,test")
    ap.add_argument("--model", default="", help="optional ONNX segmentation model for realistic widths")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    ds = Path(a.dataset)
    seg = None
    if a.model:
        import onnxruntime as ort
        s = ort.InferenceSession(a.model, providers=["CPUExecutionProvider"])
        inp = s.get_inputs()[0]
        in_h, in_w = inp.shape[2], inp.shape[3]

        def seg(bgr):
            x = cv2.resize(bgr, (in_w, in_h), interpolation=cv2.INTER_AREA)[:, :, ::-1].transpose(2, 0, 1)[None]
            sc, = s.run(None, {inp.name: np.ascontiguousarray(x, np.float32) / 255})
            best = cv2.resize(sc[0, 0], (WIDTH, HEIGHT))
            lab = np.zeros((HEIGHT, WIDTH), np.uint8)
            for c in (1, 2):
                cand = cv2.resize(sc[0, c], (WIDTH, HEIGHT))
                lab[cand > best] = c
                best = np.maximum(best, cand)
            return lab == 1

    rows = []          # (bucket, true_dist, h_rel, err_ipm_xy, err_w_xy, err_w_z, err_wm_xy, err_wm_z)
    frames_used = 0
    for split in a.splits.split(","):
        for map_dir in sorted((ds / split).iterdir()):
            wj = ds / "worlds" / f"{map_dir.name}.json"
            if not wj.is_file():
                continue                        # official track: no 3D sidecar
            meta = json.loads(wj.read_text())
            lift = meta.get("line_lift_m", 0.001)
            polys = [dense(meta["centerline"])] + [dense(m) for m in meta.get("markings", []) if len(m) > 3]
            pts = np.vstack(polys)
            pts[:, 2] += lift
            for line in (map_dir / "frames.jsonl").read_text().splitlines():
                fr = json.loads(line)
                if a.max_frames and frames_used >= a.max_frames:
                    break
                img_p = map_dir / "image" / fr["file"]
                if not img_p.is_file():
                    continue
                gt = cv2.imread(str(map_dir / "mask" / fr["file"]), cv2.IMREAD_UNCHANGED)
                gt = (gt[..., 0] if gt.ndim == 3 else gt) == 1
                pm = seg(cv2.imread(str(img_p))) if seg else None
                Rb = qmat(*fr["quaternion_xyzw"])
                pb = np.array(fr["position"])
                cam = pb + Rb @ CAM
                loc = (pts - cam) @ Rb                      # camera/body axes: x fwd, y left, z up
                ok = (loc[:, 0] > 0.31 - 0.076 + 0.0) & (loc[:, 0] < 0.85 - 0.076)
                if not ok.any():
                    continue
                frames_used += 1
                ground_self = pb[2] - BODY_Z                # robot's own ground height (world)
                nrm = Rb[:, 2]                               # robot ground-plane normal (world)
                for P, L in zip(pts[ok], loc[ok]):
                    u = int(round(CX - F * L[1] / L[0]))
                    v = int(round(CY - F * L[2] / L[0]))
                    if not (0 <= u < WIDTH and 0 <= v < HEIGHT):
                        continue
                    ray_b = np.array([1.0, -(u - CX) / F, -(v - CY) / F])
                    ray_w = Rb @ ray_b
                    # IPM: intersect with the robot's own ground plane (+ line lift)
                    plane_pt = pb - nrm * BODY_Z + nrm * lift
                    denom = ray_w @ nrm
                    if denom >= -1e-6:
                        continue
                    t = ((plane_pt - cam) @ nrm) / denom
                    p_ipm = cam + t * ray_w
                    # line direction (phi) from neighbouring IPM points along the true polyline
                    d_true = None
                    j = np.argmin(np.linalg.norm(pts - P, axis=1))
                    if 0 < j < len(pts) - 1:
                        d_true = pts[j + 1] - pts[j - 1]
                    if d_true is None:
                        continue
                    d_b = Rb.T @ d_true
                    phi = math.atan2(d_b[1], d_b[0])
                    c = abs(math.cos(phi))
                    if c < 0.6:
                        continue
                    h_rel = P[2] - lift - ground_self
                    dist = float(np.linalg.norm((P - pb)[:2]))
                    e_ipm = float(np.linalg.norm((p_ipm - P)[:2]))
                    res = [None, None, None, None]
                    for k, m in enumerate((gt, pm)):
                        if m is None:
                            continue
                        wpx = run_width(m, u, v)
                        if not wpx or wpx < 3:
                            continue
                        Z = F * LINE_W / (wpx * c)
                        p_w = cam + Rb @ (ray_b * Z)
                        res[2 * k] = float(np.linalg.norm((p_w - P)[:2]))
                        res[2 * k + 1] = float(p_w[2] - P[2])
                    bucket = "hill" if abs(h_rel) > 0.01 else "flat"
                    rows.append((bucket, dist, h_rel, e_ipm, *res))
    print(f"frames used {frames_used}, points {len(rows)}")
    out = {}
    for bucket in ("flat", "hill"):
        sel = [r for r in rows if r[0] == bucket]
        if not sel:
            continue
        res = {"points": len(sel)}
        for name, idx in (("ipm_xy", 3), ("width_gt_xy", 4), ("width_gt_z", 5), ("width_model_xy", 6), ("width_model_z", 7)):
            vals = np.array([abs(r[idx]) for r in sel if r[idx] is not None])
            if len(vals):
                res[name] = dict(n=int(len(vals)), median_cm=round(float(np.median(vals)) * 100, 2),
                                 p90_cm=round(float(np.percentile(vals, 90)) * 100, 2))
        # by distance bins
        for lo, hi in ((0.3, 0.5), (0.5, 0.7), (0.7, 0.9)):
            sub = [r for r in sel if lo <= r[1] < hi]
            if sub:
                res[f"{lo}-{hi}m"] = {n: round(float(np.median([abs(r[i]) for r in sub if r[i] is not None] or [np.nan])) * 100, 2)
                                      for n, i in (("ipm_xy_cm", 3), ("width_gt_xy_cm", 4), ("width_model_xy_cm", 6))}
        out[bucket] = res
        print(bucket, json.dumps(res))
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
