#!/usr/bin/env python3
"""Line-restoration dataset: what the robot has seen of the line -> where the line really goes.

No rendering. From the ground-truth track metadata (centre line + all painted lines) it
simulates the robot's local line map the way the node builds it, then pairs it with the true
route ahead.

Sample = robot-centric patch, x forward -0.4..1.2 m, y -0.8..0.8 m, 1.25 cm cells (128x128):
  input  ch0 seen line   : painted lines inside the camera footprints of recent poses
                           (0.31 m blind zone, ~62 deg FOV, range 0.5/0.85 m), with pose drift
                           that grows with distance travelled, missed pixels and false blobs
  input  ch1 seen area   : union of those footprints (seen-but-empty differs from unseen)
  target route           : the true centre line from 0.3 m behind to 1.4 m ahead, continuous
                           through gaps, blind zone and out-of-view parts
  extra  unseen          : target pixels never seen (where restoration matters)

  python3 experiments/restore/make_dataset.py --suites general200,official_like50,stress50,obstacles100 \
      --per-track 120 --out /path/train.npz
"""
import argparse
import json
import math
from multiprocessing import Pool
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SUITES = ROOT / "experiments/worlds/suites"
RES = 0.0125
X0, X1, Y0 = -0.4, 1.2, 0.8                       # patch extent
N = 128
X_NEAR = 0.31
HALF_FOV = math.radians(31.0)


def to_px(xy):
    """robot-frame (x fwd, y left) -> patch pixel (col, row); row 0 = far ahead."""
    col = (Y0 - xy[..., 1]) / RES
    row = (X1 - xy[..., 0]) / RES
    return np.stack([col, row], -1)


def world_to_robot(pts, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    d = pts - np.array([x, y])
    return np.stack([d[:, 0] * c + d[:, 1] * s, -d[:, 0] * s + d[:, 1] * c], -1)


def footprint(xfar):
    """camera footprint polygon in that pose's robot frame."""
    return np.array([[X_NEAR, X_NEAR * math.tan(HALF_FOV)], [xfar, xfar * math.tan(HALF_FOV)],
                     [xfar, -xfar * math.tan(HALF_FOV)], [X_NEAR, -X_NEAR * math.tan(HALF_FOV)]])


def draw_lines(canvas, lines_rob, thick=2):
    for ln in lines_rob:
        if len(ln) < 2:
            continue
        p = np.round(to_px(ln) * 8).astype(np.int32)        # 3 fractional bits
        cv2.polylines(canvas, [p], False, 255, thickness=thick, lineType=cv2.LINE_8, shift=3)


def load(path):
    m = json.loads(Path(path).read_text())
    c = np.asarray(m["centerline"], float)[:, :2]
    arc = np.asarray(m["arc_m"], float)
    painted = [np.asarray(seg, float)[:, :2] for seg in m.get("painted", [])] or [c]
    marks = [np.asarray(seg, float)[:, :2] for seg in m.get("markings", [])]
    return c, arc, painted + marks


def interp(c, arc, s):
    s = min(max(s, 0.0), arc[-1] - 1e-6)
    k = int(np.searchsorted(arc, s)) - 1
    k = max(0, min(k, len(c) - 2))
    f = (s - arc[k]) / max(arc[k + 1] - arc[k], 1e-9)
    p = c[k] + f * (c[k + 1] - c[k])
    h = math.atan2(*(c[k + 1] - c[k])[::-1])
    return p, h


def sample(track, rng):
    c, arc, paint = track
    L = arc[-1]
    s_t = rng.uniform(0.5, max(0.6, L - 0.2))
    hist = rng.uniform(0.4, 2.2)
    lat_amp = rng.uniform(0.0, 0.04)
    phase = rng.uniform(0, math.tau)

    def pose_at(s, drift=(0.0, 0.0, 0.0)):
        p, h = interp(c, arc, s)
        lat = lat_amp * math.sin(s * 3.1 + phase)
        p = p + lat * np.array([-math.sin(h), math.cos(h)])
        h = h + rng.normal(0, math.radians(2.0)) * 0.3
        return (p[0] + drift[0], p[1] + drift[1], h + drift[2])

    cur = pose_at(s_t)
    near = [ln for ln in paint if np.min(np.hypot(*(ln - cur[:2]).T)) < 2.0]
    seen_line = np.zeros((N, N), np.uint8)
    seen_area = np.zeros((N, N), np.uint8)
    # drift between when an observation was made and now (odometry error, corner jumps)
    rate = rng.uniform(0.0, 0.04)                      # m per m travelled
    yaw_rate = math.radians(rng.uniform(0.0, 1.5))     # rad per m
    jump = rng.uniform(0.0, 0.10) if rng.random() < 0.3 else 0.0
    steps = np.arange(s_t - hist, s_t + 1e-6, rng.uniform(0.03, 0.07))
    for s in steps:
        if s < 0:
            continue
        back = s_t - s
        dx, dy = rng.normal(0, rate * back + 1e-4, 2)
        if jump and back > 0.3:
            ang = rng.uniform(0, math.tau)
            dx, dy = dx + jump * math.cos(ang), dy + jump * math.sin(ang)
        obs_pose = pose_at(s)
        # observation as the robot believed it: the true view, misplaced by the drift
        xfar = 0.85 if rng.random() < 0.7 else 0.5
        fp = footprint(xfar)
        lines = [world_to_robot(ln, obs_pose) for ln in near]
        lines = [ln[(ln[:, 0] > -0.2) & (ln[:, 0] < xfar + 0.1) & (np.abs(ln[:, 1]) < 0.7)] for ln in lines]
        dyaw = rng.normal(0, yaw_rate * back + 1e-4)
        belief = (obs_pose[0] + dx, obs_pose[1] + dy, obs_pose[2] + dyaw)
        # robot frame of the observation -> world (believed) -> current robot frame
        cb, sb = math.cos(belief[2]), math.sin(belief[2])

        def to_cur(q):
            w = np.stack([belief[0] + q[:, 0] * cb - q[:, 1] * sb, belief[1] + q[:, 0] * sb + q[:, 1] * cb], -1)
            return world_to_robot(w, cur)
        mask = np.zeros((N, N), np.uint8)
        cv2.fillPoly(mask, [np.round(to_px(to_cur(fp)) * 8).astype(np.int32)], 255, shift=3)
        canvas = np.zeros((N, N), np.uint8)
        draw_lines(canvas, [to_cur(ln) for ln in lines if len(ln) > 1])
        seen_line[mask > 0] = canvas[mask > 0]          # newer views overwrite (like the map)
        seen_area |= mask
    # detector imperfections
    if rng.random() < 0.5:
        for _ in range(rng.integers(1, 4)):
            cx, cy, r = rng.integers(0, N), rng.integers(0, N), rng.integers(2, 7)
            cv2.circle(seen_line, (int(cx), int(cy)), int(r), 0, -1)
    if rng.random() < 0.3:
        for _ in range(rng.integers(1, 3)):
            cx, cy = rng.integers(0, N), rng.integers(0, N)
            cv2.ellipse(seen_line, (int(cx), int(cy)), (int(rng.integers(2, 6)), int(rng.integers(2, 10))),
                        float(rng.uniform(0, 180)), 0, 360, 255, -1)
            seen_line &= seen_area
    # target: true route around the robot, continuous through gaps
    ks = (arc > s_t - 0.3) & (arc < s_t + 1.4)
    route = np.zeros((N, N), np.uint8)
    draw_lines(route, [world_to_robot(c[ks], cur)], thick=2)
    unseen = ((route > 0) & (seen_area == 0)).astype(np.uint8)
    return seen_line, seen_area, route, unseen


def work(args):
    path, n, seed = args
    rng = np.random.default_rng(seed)
    track = load(path)
    if track[1][-1] < 1.0:
        return []
    return [sample(track, rng) for _ in range(n)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suites", required=True)
    ap.add_argument("--per-track", type=int, default=100)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workers", type=int, default=12)
    a = ap.parse_args()
    files = []
    for s in a.suites.split(","):
        files += sorted((SUITES / s / "maps").glob("*.json"))
    jobs = [(str(f), a.per_track, a.seed * 100003 + i) for i, f in enumerate(files)]
    with Pool(a.workers) as pool:
        out = [x for r in pool.imap(work, jobs, chunksize=2) for x in r]
    X = np.stack([np.stack([o[0], o[1]]) for o in out])
    Y = np.stack([o[2] for o in out])
    U = np.stack([o[3] for o in out])
    a.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(a.out, x=(X > 0).astype(np.uint8), y=(Y > 0).astype(np.uint8), unseen=U,
                        files=np.array([str(f) for f in files]))
    print(f"{len(out)} samples from {len(files)} tracks -> {a.out}  unseen route share "
          f"{U.sum() / max(1, (Y > 0).sum()):.1%}")


if __name__ == "__main__":
    main()
