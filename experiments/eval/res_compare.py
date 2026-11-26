#!/usr/bin/env python3
"""How far ahead is the line still detected: 640x480 vs 320x240 (dev only, uses ground truth).

Teleports the robot along the centreline of a tile map (every --step m, facing along the path),
grabs the camera frame, and for every ground-truth line point in view checks whether the rule
threshold (vision.line_mask's rule, in image space) marks a dark pixel there:
  640: native frame, tolerance 2 px     320: INTER_AREA downscale, tolerance 1 px (same angle)
Points are binned by forward distance and by how the line runs in the image:
  along   - line runs away from the camera (|angle to view axis| < 30 deg)
  across  - line crosses the view (> 60 deg): far side of a corner / arc, T bars, crossings
Run inside the lab container (gz sim of the map must NOT be running; this starts its own):
  python3 experiments/eval/res_compare.py --map experiments/worlds/suites/tiles60/maps/tile_034.sdf
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/line_follower"))
from line_follower import vision  # noqa: E402

BINS = np.arange(0.3, 2.61, 0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", type=Path, required=True)
    ap.add_argument("--step", type=float, default=0.1)
    ap.add_argument("--vtol", type=int, default=6, help="vertical tolerance at 640 px (pose pitch jitter)")
    ap.add_argument("--out", type=Path, default=ROOT / "experiments/results/lead/res_compare")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    meta = json.loads(a.map.with_suffix(".json").read_text())
    C = np.array([p[:2] for p in meta["centerline"]])
    lines = [C] + [np.array([p[:2] for p in m]) for m in meta["markings"]]
    # dense GT points (every 1 cm) with local direction
    gt_pts, gt_dir = [], []
    for L in lines:
        for p, q in zip(L[:-1], L[1:]):
            d = np.linalg.norm(q - p)
            n = max(1, int(d / 0.01))
            for k in range(n):
                gt_pts.append(p + (q - p) * k / n)
                gt_dir.append(math.atan2(q[1] - p[1], q[0] - p[0]))
    gt_pts, gt_dir = np.array(gt_pts), np.array(gt_dir)

    env = dict(os.environ, GZ_PARTITION=f"rescmp_{os.getpid()}",
               GZ_SIM_RESOURCE_PATH=str(ROOT / "src/line_follower_bringup/models"))
    os.environ.update(env)
    sim = subprocess.Popen(["gz", "sim", "-s", "-r", "--headless-rendering", str(a.map)],
                           stdout=open(a.out / "gz.log", "w"), stderr=subprocess.STDOUT, env=env)
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.boolean_pb2 import Boolean
    node = Node()
    frames = []

    def on_img(m):
        frames.append((time.monotonic(), np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).copy()))
        del frames[:-3]
    node.subscribe(Image, "/camera/image_raw", on_img)
    t = time.monotonic()
    while not frames and time.monotonic() - t < 60:
        time.sleep(0.2)
    assert frames, "no camera frames"
    world = meta["world"]

    def set_pose(x, y, yaw):
        req = Pose()
        req.name = "turtlebot3_waffle_pi"
        req.position.x, req.position.y, req.position.z = x, y, 0.01
        req.orientation.z, req.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        node.request(f"/world/{world}/set_pose", req, Pose, Boolean, 2000)

    arc = np.array(meta["arc_m"])
    hills = meta.get("ramps", [])
    stats = {r: {k: np.zeros((len(BINS), 2)) for k in ("along", "across", "all")} for r in (640, 320)}
    examples = []
    s = 0.2
    nposes = 0
    while s < arc[-1] - 0.3:
        i = int(np.searchsorted(arc, s))
        x, y = C[i]
        j = min(len(C) - 1, i + 3)
        yaw = math.atan2(C[j][1] - C[max(0, i - 3)][1], C[j][0] - C[max(0, i - 3)][0])
        s += a.step
        if any(abs(x - r["x0"]) < 1.8 and abs(y - r["y0"]) < 1.8 for r in hills):
            continue                                  # flat ground only
        set_pose(x, y, yaw)
        t_set = time.monotonic()
        time.sleep(0.35)
        while not frames or frames[-1][0] < t_set + 0.2:
            time.sleep(0.02)
        rgb = frames[-1][1]
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        nposes += 1
        # GT points in the robot frame
        c, sn = math.cos(yaw), math.sin(yaw)
        rel = gt_pts - [x, y]
        rx = rel[:, 0] * c + rel[:, 1] * sn
        ry = -rel[:, 0] * sn + rel[:, 1] * c
        ang = np.abs(((gt_dir - yaw + math.pi / 2) % math.pi) - math.pi / 2)   # 0 = along view
        keep = (rx > vision.CAM_X + 0.05) & (rx < BINS[-1] + 0.1)
        for res, tol in ((640, 2), (320, 1)):
            g = gray if res == 640 else cv2.resize(gray, (320, 240), interpolation=cv2.INTER_AREA)
            h, w = g.shape
            ground = g[int(h * 0.55):]
            thr = min(125.0, 0.62 * float(np.median(ground)))
            mask = (g < thr).astype(np.uint8)
            vt = max(tol, a.vtol * res // 640)
            mask = cv2.dilate(mask, np.ones((2 * vt + 1, 2 * tol + 1), np.uint8))
            k = res / 640.0
            for idx in np.where(keep)[0]:
                u, v = vision.ground_to_image(rx[idx], ry[idx])
                u, v = u * k, v * k
                if not (0 <= u < w and 0 <= v < h):
                    continue
                b = int((rx[idx] - BINS[0]) / 0.1)
                if b < 0 or b >= len(BINS):
                    continue
                hit = mask[int(round(v)) if round(v) < h else h - 1, int(round(u)) if round(u) < w else w - 1]
                kinds = ["all"] + (["along"] if ang[idx] < math.radians(30) else
                                   ["across"] if ang[idx] > math.radians(60) else [])
                for kd in kinds:
                    stats[res][kd][b] += (hit, 1)
        far_across = keep & (ang > math.radians(60)) & (rx > 0.95) & (rx < 1.35) & (np.abs(ry) < 0.4)
        if far_across.sum() > 15 and len(examples) < 6 and nposes % 3 == 0:
            examples.append(rgb)
    sim.terminate()
    out = {"map": str(a.map), "poses": nposes, "bins_m": [round(float(b), 2) for b in BINS]}
    for res in (640, 320):
        for kd in ("along", "across", "all"):
            st = stats[res][kd]
            out[f"{res}_{kd}_recall"] = [round(float(h / n), 3) if n else None for h, n in st]
            out[f"{res}_{kd}_n"] = [int(n) for _, n in st]
    name = a.map.stem
    (a.out / f"{name}.json").write_text(json.dumps(out))
    for k, im in enumerate(examples):
        cv2.imwrite(str(a.out / f"{name}_ex{k}.png"), cv2.cvtColor(im, cv2.COLOR_RGB2BGR))
    print(name, "poses", nposes)
    for kd in ("along", "across"):
        print(kd)
        for b, r6, r3, n in zip(BINS, out[f"640_{kd}_recall"], out[f"320_{kd}_recall"], out[f"640_{kd}_n"]):
            print(f"  {b:.1f} m  640 {r6}  320 {r3}  n={n}")


if __name__ == "__main__":
    main()
