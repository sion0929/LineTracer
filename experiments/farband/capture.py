#!/usr/bin/env python3
"""Far-band dataset (dev only): full-resolution horizon strips of the 640x480 camera + exact line masks.

For each tile map: start gz sim, teleport the robot to random poses near the line (flat ground,
away from ramps), grab the camera frame and keep rows ROW0..ROW1 (~0.8-1.9 m ahead). The label is
the fraction of each pixel's ground footprint covered by the 2 cm painted line (centre line +
T bars), computed from the camera model with 2x2 supersampling. Run inside the lab container:
  python3 experiments/farband/capture.py --maps LIST --out DIR --poses 150 --seed 0
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/line_follower"))
from line_follower import vision  # noqa: E402

ROW0, ROW1 = 256, 320          # strip rows of the 480-row image (horizon at 239.5)
SS = 2                          # supersampling per axis


def dense(poly, step=0.0025):
    poly = np.asarray(poly, float)[:, :2]
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
    return np.array(out)


def ground_grid():
    """Robot-frame ground (x, y) of each sub-pixel of the strip, shape (H*SS, W*SS, 2)."""
    vs = ROW0 + (np.arange((ROW1 - ROW0) * SS) + 0.5) / SS - 0.5
    us = (np.arange(vision.W * SS) + 0.5) / SS - 0.5
    zc = vision.FX * vision.CAM_H / (vs - vision.CY)          # forward distance from the camera
    x = zc[:, None] + vision.CAM_X + 0 * us[None, :]
    y = -(us[None, :] - vision.CX) * zc[:, None] / vision.FX
    return np.stack([x, y], -1)


def ramp_h(ramps, X, Y, width=0.66):
    """Vectorised ground height of the generator ramps at world points X, Y (arrays)."""
    h = np.zeros_like(X)
    for r in ramps:
        c, s = math.cos(r["heading"]), math.sin(r["heading"])
        u = (X - r["x0"]) * c + (Y - r["y0"]) * s
        v = -(X - r["x0"]) * s + (Y - r["y0"]) * c
        up, deck, down, hh = r["up"], r["deck"], r["down"], r["height_m"]
        L = up + deck + down
        z = np.where(u < up, hh * u / up, np.where(u < up + deck, hh, hh * (L - u) / down))
        z = np.where((u > 0) & (u < L) & (np.abs(v) <= width / 2), z, 0.0)
        h = np.maximum(h, z)
    return h


def raycast(x, y, yaw, ramps, step=0.004):
    """World ground point hit by each strip pixel ray (camera level, robot on flat floor)."""
    vs = ROW0 + np.arange(ROW1 - ROW0)
    us = np.arange(vision.W)
    V, U = np.meshgrid(vs, us, indexing="ij")
    dz = (V - vision.CY) / vision.FX                     # drop per metre forward
    dy = -(U - vision.CX) / vision.FX
    t_hi = vision.CAM_H / dz                             # flat floor hit
    t_lo = np.maximum(0.0, (vision.CAM_H - 0.08) / dz)   # above any ramp before this
    c, s = math.cos(yaw), math.sin(yaw)
    hit = t_hi.copy()
    # only rays that pass over a ramp footprint need marching; the rest hit the flat floor
    near = np.zeros(dz.shape, bool)
    for f in np.linspace(0.0, 1.0, 24):
        ts = t_lo + f * (t_hi - t_lo)
        xr = vision.CAM_X + ts; yr = dy * ts
        near |= ramp_h(ramps, x + xr * c - yr * s, y + xr * s + yr * c, width=0.80) > 0
    done = ~near
    tt = t_lo.copy()
    while True:
        act = ~done & (tt < t_hi)
        if not act.any():
            break
        xr = vision.CAM_X + tt; yr = dy * tt; z = vision.CAM_H - dz * tt
        X = x + xr * c - yr * s; Y = y + xr * s + yr * c
        h = ramp_h(ramps, X, Y)
        new = act & (z <= h)
        hit[new] = tt[new]; done |= new
        tt = tt + step
    xr = vision.CAM_X + hit; yr = dy * hit
    return x + xr * c - yr * s, y + xr * s + yr * c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--maps", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--poses", type=int, default=150)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--hills", action="store_true", help="poses on flat floor looking at the ramps; ray-cast labels")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(a.seed)
    G = ground_grid()
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.boolean_pb2 import Boolean
    for line in a.maps.read_text().split():
        sdf = ROOT / line
        name = sdf.stem
        if (a.out / f"{name}.npz").exists():
            continue
        meta = json.loads(sdf.with_suffix(".json").read_text())
        C = np.array([p[:2] for p in meta["centerline"]])
        arc = np.array(meta["arc_m"])
        gt = np.vstack([dense(C)] + [dense(m) for m in meta["markings"] if len(m) > 1])
        tree = cKDTree(gt)
        env = dict(os.environ, GZ_PARTITION=f"farcap_{os.getpid()}_{name}",
                   GZ_SIM_RESOURCE_PATH=str(ROOT / "src/line_follower_bringup/models"))
        os.environ.update(env)
        sim = subprocess.Popen(["gz", "sim", "-s", "-r", "--headless-rendering", str(sdf)],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
        node = Node()
        frames = []

        def on_img(m, frames=frames):
            frames.append((time.monotonic(), np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).copy()))
            del frames[:-2]
        node.subscribe(Image, "/camera/image_raw", on_img)
        t = time.monotonic()
        while not frames and time.monotonic() - t < 90:
            time.sleep(0.2)
        if not frames:
            sim.kill(); print("no frames", name); continue
        ramps = meta.get("ramps", [])
        X, Y, P = [], [], []
        tries = 0
        while len(X) < a.poses and tries < a.poses * 4:
            tries += 1
            s = rng.uniform(0.1, arc[-1] - 0.1)
            i = int(np.searchsorted(arc, s))
            x, y = C[i]
            near_ramp = [r for r in ramps if math.hypot(x - r["x0"], y - r["y0"]) < 2.6]
            if a.hills:
                # robot on the flat floor (wheels too), the ramp ahead within ~0.6-2 m
                if not near_ramp:
                    continue
                if ramp_h(ramps, np.array([x, x + 0.15, x - 0.15, x]), np.array([y, y, y, y + 0.15])).max() > 0:
                    continue
            elif any(math.hypot(x - r["x0"], y - r["y0"]) < 2.0 for r in ramps):
                continue
            j0, j1 = max(0, i - 3), min(len(C) - 1, i + 3)
            yaw = math.atan2(C[j1][1] - C[j0][1], C[j1][0] - C[j0][0])
            yaw += rng.normal(0, math.radians(12)) if rng.random() < 0.92 else rng.uniform(-math.pi, math.pi)
            off = rng.normal(0, 0.04)
            x, y = x - off * math.sin(yaw), y + off * math.cos(yaw)
            req = Pose(); req.name = "turtlebot3_waffle_pi"
            req.position.x, req.position.y, req.position.z = x, y, 0.01
            req.orientation.z, req.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            node.request(f"/world/{meta['world']}/set_pose", req, Pose, Boolean, 2000)
            t_set = time.monotonic()
            while time.monotonic() - t_set < 3 and (not frames or frames[-1][0] < t_set + 0.25):
                time.sleep(0.02)
            rgb = frames[-1][1]
            if a.hills:
                if ramp_h(ramps, np.array([x]), np.array([y]))[0] > 0:
                    continue
                wx, wy = raycast(x, y, yaw, ramps)
                d = tree.query(np.stack([wx.ravel(), wy.ravel()], 1), distance_upper_bound=0.02)[0]
                cov = (d <= 0.01).reshape(wx.shape).astype(np.float32)
            else:
                c, sn = math.cos(yaw), math.sin(yaw)
                wx = x + G[..., 0] * c - G[..., 1] * sn
                wy = y + G[..., 0] * sn + G[..., 1] * c
                d = tree.query(np.stack([wx.ravel(), wy.ravel()], 1), distance_upper_bound=0.02)[0]
                cov = (d <= 0.01).reshape(wx.shape).astype(np.float32)
                cov = cov.reshape(ROW1 - ROW0, SS, vision.W, SS).mean((1, 3))
            X.append(rgb[ROW0:ROW1]); Y.append((cov * 255).astype(np.uint8)); P.append((x, y, yaw))
        sim.terminate()
        try:
            sim.wait(10)
        except subprocess.TimeoutExpired:
            sim.kill()
        np.savez_compressed(a.out / f"{name}.npz", x=np.array(X), y=np.array(Y), pose=np.array(P),
                            rows=np.array([ROW0, ROW1]))
        print(name, len(X), flush=True)


if __name__ == "__main__":
    main()
