#!/usr/bin/env python3
"""Record camera frames + ground truth on ONE map for perception tests (dev tool, Gazebo).

Uses Gazebo set_pose (development only; the robot runtime never does this). The robot is
teleported to sampled arc positions along the ordered centreline with small lateral / yaw
perturbations (plus a share of positions on and in front of hills), allowed to settle, and the
camera frame is stored with the TRUE robot pose and the true line masks.

  # official track (default world), 40 frames, 30 % of them around the hill
  python3 experiments/eval/record_perception.py --out /tmp/official.npz
  # any generated map (SDF with its JSON/STL sidecars next to it)
  python3 experiments/eval/record_perception.py --world experiments/worlds/suites/hills60/maps/easy_9000.sdf \
      --count 40 --out /tmp/easy_9000.npz
  # fixed number of frames per scene type (used for the package test fixture)
  python3 experiments/eval/record_perception.py --quota straight=6,corner=6,crossing=5,loop=4,hill=7 \
      --seed 7 --out src/line_follower/test/data/official_track_rec.npz

Isolation: run inside the lab container. Gazebo gets its own GZ_PARTITION (default
lf_p_rec_<pid>) and ROS_DOMAIN_ID (default 60; nothing ROS is started). Reuses
experiments/dl/collect.py (simulation, Capture.teleport, geometry loaders) and
experiments/dl/geometry.py (render_mask).

Output: one np.savez_compressed file, loadable with np.load(path) (no pickle):
  frames_png, frames_off   camera frames (BGR, PNG-encoded), concatenated bytes + offsets
  gtmask_png, gtmask_off   true image masks (0 background, 1 line, 2 obstacle), PNG-encoded
  gt_bev                   np.packbits of the true VISIBLE line in the package BEV grid
                           (vision.py: row = (X_FAR - x) / RES, col = (Y_HALF - y) / RES, robot
                           body frame, x from the axle); shape in bev_shape
  poses                    (N, 6) true model pose x, y, z, roll, pitch, yaw (Gazebo convention:
                           pitch > 0 = nose DOWN; the IMU-style nose-up pitch is -pitch)
  quat                     (N, 4) x, y, z, w;   stamp (N,) sim time of the frame
  arc_s, lateral_off, yaw_off_deg   where the pose was sampled (route arc, perturbations)
  probe_x (3,), probe_y (N, 3)  lateral position [m, body frame] of the route ahead where it first
                           crosses x = probe_x, NaN when not visible / painted, or when the route
                           runs within 30 deg of sideways there (centre ill defined)
  route_body, route_off    route (2 cm steps, arc_s - 0.3 .. arc_s + 1.8 m) in the body frame
  on_hill, sees_hill, hill (N,) bool;  category (N,) primary scene type;  tags (N,) all types
  meta                     JSON string: map, world, generator info, capture settings, BEV grid
"""
import argparse
import csv
import json
import math
import os
import secrets
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src/line_follower"))

from line_follower import vision  # noqa: E402

OFFICIAL = ROOT / "src/line_follower_bringup/worlds/line_track.sdf"
PROBE_X = (0.35, 0.50, 0.70)
CATEGORIES = ("hill", "loop", "crossing", "corner", "straight")   # priority order for the primary type
ROUTE_BACK, ROUTE_AHEAD, ROUTE_STEP = 0.3, 1.8, 0.02
MAX_PROBE_ANGLE = math.radians(60.0)
LOOP_ARC = 1.5                            # self-crossing within this much arc = small loop
SUB = 4                                   # GT raster sub-cells per BEV cell side


# ------------------------------------------------------------------ route + tags
class Route:
    """Ordered centreline resampled at 1 cm: xyz (ground height in z), heading, arc."""

    def __init__(self, xyz, arc):
        xyz, arc = np.asarray(xyz, float), np.asarray(arc, float)
        keep = np.r_[True, np.diff(arc) > 1e-9]
        xyz, arc = xyz[keep], arc[keep]
        self.length = float(arc[-1])
        self.s = np.arange(0.0, self.length, 0.01)
        self.xyz = np.column_stack([np.interp(self.s, arc, xyz[:, k]) for k in range(3)])
        # heading over a 6 cm chord: robust to the few-mm overshoots at square corners in gt.npy
        k = 3
        pad = np.vstack([self.xyz[:1, :2]] * k + [self.xyz[:, :2]] + [self.xyz[-1:, :2]] * k)
        d = pad[2 * k:] - pad[:-2 * k]
        self.heading = np.arctan2(d[:, 1], d[:, 0])
        self.turn = np.r_[0.0, np.abs(np.diff(np.unwrap(self.heading)))]   # |turn| per 1 cm step
        self.cum_turn = np.cumsum(self.turn)

    def index(self, s):
        return int(np.clip(round(s / 0.01), 0, len(self.s) - 1))

    def total_turn(self, a, b):
        return float(self.cum_turn[self.index(b)] - self.cum_turn[self.index(a)])

    def slope(self, s, span=0.05):
        i0, i1 = self.index(s - span), self.index(s + span)
        run = max(self.s[i1] - self.s[i0], 1e-6)
        return math.atan2(self.xyz[i1, 2] - self.xyz[i0, 2], run)


def to_body(points, position, rotation):
    return (np.asarray(points, float) - position) @ rotation


def nominal_frame(route, s):
    i = route.index(s)
    yaw = route.heading[i]
    c, si = math.cos(yaw), math.sin(yaw)
    return route.xyz[i], np.array([[c, -si, 0.0], [si, c, 0.0], [0.0, 0.0, 1.0]])


def scene_tags(route, s, others):
    """Scene types seen from a robot standing on the route at arc s, looking along it.

    hill: the ground under / ahead of the robot is not flat.
    loop: the route crosses itself within LOOP_ARC of arc just ahead, or turns >= 300 deg in
          total within 1 m (small circle, official-track bulb).
    crossing: another part of the route or a marking (bar, stub, T) meets the route ahead.
    corner: >= 40 deg of turning in view.   straight: none of loop / crossing / corner."""
    tags = set()
    lo, hi = route.index(s - 0.15), route.index(s + 0.9)
    if np.abs(route.xyz[lo:hi + 1, 2]).max() > 0.004:
        tags.add("hill")
    a, b = route.index(s + 0.2), route.index(s + 0.9)
    ahead_s = route.s[a:b + 1:2]
    ahead = route.xyz[a:b + 1:2, :2]
    sep = np.abs(route.s[None, :] - ahead_s[:, None])
    touch = (np.linalg.norm(route.xyz[None, :, :2] - ahead[:, None], axis=2) < 0.03) & (sep > 0.3)
    if np.any(touch & (sep <= LOOP_ARC)) or route.total_turn(s, s + 1.0) >= math.radians(300):
        tags.add("loop")
    if np.any(touch & (sep > LOOP_ARC)) or (len(others) and np.linalg.norm(
            others[None, :, :2] - ahead[:, None], axis=2).min() < 0.03):
        tags.add("crossing")
    if "loop" not in tags and route.total_turn(s + 0.1, s + 0.85) >= math.radians(40):
        tags.add("corner")
    if not tags - {"hill"}:
        tags.add("straight")
    return tags


def primary(tags):
    return next(c for c in CATEGORIES if c in tags)


# ------------------------------------------------------------------ sampling
def candidate_grid(route, others, step=0.02):
    s_all = np.arange(0.15, route.length - 0.35, step)
    return [(float(s), scene_tags(route, s, others)) for s in s_all]


def far_enough(s, chosen, sep):
    return all(abs(s - c) >= sep for c in chosen)


def draw_positions(route, others, rng, count, hill_frac, quota, min_sep, keepout):
    """Arc positions (s, tags). quota (dict) overrides count / hill_frac."""
    grid = [(s, t) for s, t in candidate_grid(route, others) if not keepout(route, s)]
    chosen = []
    if quota:
        for cat, n in quota.items():
            pool = [g for g in grid if primary(g[1]) == cat] or [g for g in grid if cat in g[1]]
            order = rng.permutation(len(pool))
            got = 0
            for k in order:
                s, tags = pool[k]
                if got < n and far_enough(s, [c[0] for c in chosen], min_sep):
                    chosen.append((s, tags))
                    got += 1
            if got < n:
                print(f"warning: only {got}/{n} '{cat}' positions on this map", flush=True)
        return chosen
    hills = [g for g in grid if "hill" in g[1]]
    n_hill = min(int(round(count * hill_frac)), len(hills)) if hills else 0
    for k in rng.permutation(len(hills)):
        if len(chosen) >= n_hill:
            break
        if far_enough(hills[k][0], [c[0] for c in chosen], min_sep):
            chosen.append(hills[k])
    n_rest = count - len(chosen)
    ss = np.array([g[0] for g in grid])
    for j in range(n_rest):               # jittered uniform arc positions
        target = (j + rng.random()) * route.length / max(n_rest, 1)
        order = np.argsort(np.abs(ss - target))
        for k in order[:200]:
            if far_enough(ss[k], [c[0] for c in chosen], min_sep * 0.5):
                chosen.append(grid[k])
                break
    return chosen


# ------------------------------------------------------------------ ground truth
def bev_coords(xy):
    """Body-frame (x, y) -> BEV (col, row) in the package convention (vision.bev_to_robot)."""
    return np.stack(((vision.Y_HALF - xy[..., 1]) / vision.RES, (vision.X_FAR - xy[..., 0]) / vision.RES), -1)


def gt_bev(line_triangles, position, rotation, depth, focal, center, cam_offset):
    """True VISIBLE painted line in the BEV grid (body frame, package cell convention).

    Every BEV cell covered by the line mesh footprint gets the 3D height of its triangle; the
    cell counts as seen when that surface point projects inside the image and nothing in the
    rendered depth buffer (terrain crest, box, ...) is clearly in front of it."""
    tris = to_body(line_triangles.reshape(-1, 3), position, rotation).reshape(-1, 3, 3)
    near = ((tris[:, :, 0] > vision.X_NEAR - 0.15) & (tris[:, :, 0] < vision.X_FAR + 0.15)
            & (np.abs(tris[:, :, 1]) < vision.Y_HALF + 0.15)).any(1)
    tris = tris[near]
    # rasterise at SUB x SUB sub-cells; a BEV cell is line when >= half of it is visible line
    ids = np.zeros((vision.BH * SUB, vision.BW * SUB), np.int32)   # triangle index + 1
    for k, tri in enumerate(tris):
        pts = np.round(((bev_coords(tri[:, :2]) + 0.5) * SUB - 0.5) * 8).astype(np.int32)
        cv2.fillConvexPoly(ids, pts, k + 1, lineType=cv2.LINE_8, shift=3)
    rows, cols = np.nonzero(ids)
    if not len(rows):
        return np.zeros((vision.BH, vision.BW), bool)
    tri = tris[ids[rows, cols] - 1]
    x = vision.X_FAR - ((rows + 0.5) / SUB - 0.5) * vision.RES
    y = vision.Y_HALF - ((cols + 0.5) / SUB - 0.5) * vision.RES
    normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    flat = np.abs(normal[:, 2]) > 1e-12
    z = np.where(flat, tri[:, 0, 2] - (normal[:, 0] * (x - tri[:, 0, 0]) + normal[:, 1] * (y - tri[:, 0, 1]))
                 / np.where(flat, normal[:, 2], 1.0), tri[:, :, 2].mean(1))
    fwd, side, up = x - cam_offset[0], y - cam_offset[1], z - cam_offset[2]
    ok = fwd > 0.05
    u = np.round(center[0] - focal * side / np.where(ok, fwd, 1.0)).astype(int)
    v = np.round(center[1] - focal * up / np.where(ok, fwd, 1.0)).astype(int)
    ok &= (u >= 0) & (u < depth.shape[1]) & (v >= 0) & (v < depth.shape[0])
    seen = np.zeros_like(ok)
    d = depth[v[ok], u[ok]]
    seen[ok] = (d > 0) & (d >= fwd[ok] - np.maximum(0.02, 0.1 * fwd[ok]))
    fine = np.zeros(ids.shape, np.float32)
    fine[rows[seen], cols[seen]] = 1.0
    return fine.reshape(vision.BH, SUB, vision.BW, SUB).mean(axis=(1, 3)) >= 0.5


def probe_lateral(route_body_fine, start, bev, probes=PROBE_X):
    """y of the route where it FIRST crosses x = probe (walking forward from index start)."""
    out = np.full(len(probes), np.nan)
    pts = route_body_fine
    for k, x0 in enumerate(probes):
        dx = pts[start:, 0] - x0
        cross = np.nonzero((dx[:-1] < 0) != (dx[1:] < 0))[0]
        if not len(cross):
            continue
        i = start + cross[0]
        a, b = pts[i], pts[i + 1]
        if abs(math.atan2(b[1] - a[1], b[0] - a[0])) > MAX_PROBE_ANGLE:
            continue                       # line runs (nearly) sideways or backwards here
        t = (x0 - a[0]) / (b[0] - a[0])
        y = a[1] + t * (b[1] - a[1])
        col, row = bev_coords(np.array([x0, y]))
        r0, c0 = int(round(row)), int(round(col))
        if not (1 <= r0 < vision.BH - 1 and 2 <= c0 < vision.BW - 2):
            continue
        if bev[r0 - 1:r0 + 2, c0 - 2:c0 + 3].any():   # painted and visible there
            out[k] = y
    return out


def euler(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


# ------------------------------------------------------------------ map loading
def suite_row(world):
    for index in (ROOT / "experiments/worlds/suites").glob("*/index.csv"):
        with index.open() as f:
            for row in csv.DictReader(f):
                if Path(row["path"]).name == world.name and (ROOT / row["path"]).resolve() == world.resolve():
                    return index.parent.name, row
    return None, None


def load_map(world):
    from experiments.dl import collect

    world = Path(world).resolve()
    if world == OFFICIAL.resolve():
        meshes, path_xyz = collect.official_geometry()
        arc = np.load(ROOT / "experiments/gt.npy")[:, 2]
        info = dict(source="official", world_name="line_track")
        others = np.zeros((0, 3))
        boxes = []
    else:
        meshes, path_xyz, meta = collect.random_geometry(world)
        arc = np.asarray(meta["arc_m"], float)
        suite, row = suite_row(world)
        info = dict(source="generated", world_name=meta["world"], seed=meta.get("seed"),
                    difficulty=meta.get("difficulty"), mirror=bool(meta.get("mirror", False)),
                    hills=len(meta.get("ramps", [meta["ramp"]])), obstacles=len(meta["obstacles"]),
                    suite=suite, suite_row=row)
        others = np.array([p for m in meta["markings"] for p in m], float).reshape(-1, 3)
        boxes = meta["obstacles"]
    return world, meshes, np.asarray(path_xyz, float), arc, info, others, boxes


def make_keepout(boxes):
    centers = np.array([b["pose"][:2] for b in boxes]).reshape(-1, 2)
    radii = np.array([np.linalg.norm(b["size"][:2]) / 2 + 0.25 for b in boxes])

    def keepout(route, s):
        if not len(centers):
            return False
        xy = route.xyz[route.index(s), :2]
        return bool(np.any(np.linalg.norm(centers - xy, axis=1) < radii))
    return keepout


# ------------------------------------------------------------------ main
def record(world, out, count=40, hill_frac=0.3, quota=None, seed=None, lateral=0.03, yaw_deg=8.0,
           settle=0.5, min_sep=0.12, partition=None, domain=60, png_level=9):
    from experiments.dl import collect
    from experiments.dl.geometry import CAMERA_OFFSET, CENTER, FOCAL, quaternion_matrix, render_mask

    seed = secrets.randbelow(2 ** 31) if seed is None else int(seed)
    rng = np.random.default_rng(seed)
    world, meshes, path_xyz, arc, info, others, boxes = load_map(world)
    route = Route(path_xyz, arc)
    positions = draw_positions(route, others, rng, count, hill_frac, quota, min_sep, make_keepout(boxes))
    if not positions:
        raise RuntimeError("no positions to record")
    line_tris = np.concatenate([m for m, label in meshes if label == 1]).astype(float)
    partition = partition or f"lf_p_rec_{os.getpid()}"
    env = {"LAB_NAME": os.environ.get("LAB_NAME", "lf_p"), "ROS_DOMAIN_ID": str(domain), "GZ_PARTITION": partition}
    collect.ENV = env                 # collect.simulation() merges this into the gz sim environment
    os.environ.update(env)            # the gz.transport Node of this process uses the same partition
    cv2.setNumThreads(1)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    t_start = time.time()
    print(f"record {world.name}: {len(positions)} poses, seed {seed}, partition {partition}", flush=True)
    log = Path(tempfile.gettempdir()) / f"{partition}.gazebo.log"
    print(f"gazebo log: {log}", flush=True)
    with collect.simulation(world, log):
        capture = collect.Capture(info["world_name"])
        try:
            capture.synchronized(0, 90)
            for n, (s, tags) in enumerate(positions):
                i = route.index(s)
                heading = route.heading[i]
                off = float(rng.uniform(-lateral, lateral))
                dyaw = float(rng.uniform(-yaw_deg, yaw_deg))
                xyz = route.xyz[i].copy()
                xyz[:2] += off * np.array([-math.sin(heading), math.cos(heading)])
                xyz[2] += 0.03
                yaw = heading + math.radians(dyaw)
                pitch = -route.slope(s) * math.cos(math.radians(dyaw))
                for attempt in range(4):
                    try:
                        stamp, message, pose = capture.teleport(xyz, yaw, pitch, settle)
                        break
                    except (TimeoutError, RuntimeError) as error:   # slow start / dropped request
                        print(f"  retry {s:.2f} m: {error}", flush=True)
                        time.sleep(1.0)
                else:
                    print(f"  skip {s:.2f} m", flush=True)
                    continue
                position = np.array([pose.position.x, pose.position.y, pose.position.z])
                q = pose.orientation
                quat = (q.x, q.y, q.z, q.w)
                rotation = quaternion_matrix(*quat)
                mask, depth = render_mask(meshes, position, rotation, return_depth=True)
                bgr = collect.to_bgr(message)
                bev = gt_bev(line_tris, position, rotation, depth, FOCAL, CENTER, CAMERA_OFFSET)
                lo, hi = route.index(s - ROUTE_BACK), route.index(s + ROUTE_AHEAD)
                fine = to_body(route.xyz[lo:hi + 1], position, rotation)
                probes = probe_lateral(fine, i - lo, bev)
                coarse = fine[::int(round(ROUTE_STEP / 0.01))]
                roll, pit, yw = euler(quat)
                view = (fine[:, 0] > 0.25) & (fine[:, 0] < 0.90) & (np.abs(fine[:, 1]) < 0.5)
                ground_here = route.xyz[i, 2]
                on_hill = abs(pit) > math.radians(1.5) or ground_here > 0.004
                sees_hill = bool(np.any(np.abs(route.xyz[lo:hi + 1, 2][view] - ground_here) > 0.004))
                rows.append(dict(
                    png=cv2.imencode(".png", bgr, [cv2.IMWRITE_PNG_COMPRESSION, png_level])[1].tobytes(),
                    mask=cv2.imencode(".png", mask, [cv2.IMWRITE_PNG_COMPRESSION, 9])[1].tobytes(),
                    bev=np.packbits(bev.ravel()), pose=(*position, roll, pit, yw), quat=quat, stamp=stamp,
                    s=s, off=off, dyaw=dyaw, probes=probes, route=coarse.astype(np.float32),
                    on_hill=on_hill, sees_hill=sees_hill, category=primary(tags), tags="+".join(sorted(tags))))
                print(f"  {n + 1:3d}/{len(positions)} s={s:6.2f} {primary(tags):8} pitch {math.degrees(-pit):+5.1f}° "
                      f"z {position[2]:.3f} probes {np.round(probes, 3).tolist()}", flush=True)
        finally:
            capture.close()
    if not rows:
        raise RuntimeError("no frame recorded")
    save(out, rows, dict(
        tool="experiments/eval/record_perception.py", created=time.strftime("%Y-%m-%d %H:%M:%S"),
        map=world.stem, world=str(world.relative_to(ROOT)) if world.is_relative_to(ROOT) else str(world),
        generator=info, capture_seed=seed, count=len(rows), settle_sec=settle, lateral_m=lateral,
        yaw_deg=yaw_deg, hill_frac=None if quota else hill_frac, quota=quota, min_sep_m=min_sep,
        wall_sec=round(time.time() - t_start, 1),
        bev=dict(res=vision.RES, x_near=vision.X_NEAR, x_far=vision.X_FAR, y_half=vision.Y_HALF,
                 shape=[vision.BH, vision.BW],
                 convention="row = (x_far - x)/res, col = (y_half - y)/res; body frame, x from the axle"),
        camera=dict(width=vision.W, height=vision.H, hfov=vision.HFOV, cam_x=vision.CAM_X, cam_h=vision.CAM_H),
        probe_x=list(PROBE_X), route=dict(back_m=ROUTE_BACK, ahead_m=ROUTE_AHEAD, step_m=ROUTE_STEP),
        pose_convention="x y z roll pitch yaw of the model origin (axle centre, 1 cm above ground on flat); "
                        "Gazebo pitch > 0 = nose down, IMU nose-up pitch = -pitch",
    ))
    print(f"wrote {out} ({out.stat().st_size / 1e6:.2f} MB, {len(rows)} frames, "
          f"{time.time() - t_start:.0f} s)", flush=True)
    return out


def _concat(chunks):
    off = np.zeros(len(chunks) + 1, np.int64)
    off[1:] = np.cumsum([len(c) for c in chunks])
    return np.frombuffer(b"".join(chunks), np.uint8), off


def save(out, rows, meta):
    frames, frames_off = _concat([r["png"] for r in rows])
    masks, masks_off = _concat([r["mask"] for r in rows])
    route, route_off = _concat([r["route"].tobytes() for r in rows])
    np.savez_compressed(
        out, frames_png=frames, frames_off=frames_off, gtmask_png=masks, gtmask_off=masks_off,
        gt_bev=np.stack([r["bev"] for r in rows]), bev_shape=np.array([vision.BH, vision.BW]),
        poses=np.array([r["pose"] for r in rows]), quat=np.array([r["quat"] for r in rows]),
        stamp=np.array([r["stamp"] for r in rows]), arc_s=np.array([r["s"] for r in rows]),
        lateral_off=np.array([r["off"] for r in rows]), yaw_off_deg=np.array([r["dyaw"] for r in rows]),
        probe_x=np.array(PROBE_X), probe_y=np.array([r["probes"] for r in rows]),
        route_body=route.view(np.float32).reshape(-1, 3), route_off=route_off // 12,
        on_hill=np.array([r["on_hill"] for r in rows]), sees_hill=np.array([r["sees_hill"] for r in rows]),
        hill=np.array([r["on_hill"] or r["sees_hill"] for r in rows]),
        category=np.array([r["category"] for r in rows]), tags=np.array([r["tags"] for r in rows]),
        meta=np.array(json.dumps(meta)),
    )


def parse_quota(text):
    if not text:
        return None
    quota = {}
    for part in text.split(","):
        cat, n = part.split("=")
        if cat not in CATEGORIES:
            raise ValueError(f"unknown scene type {cat!r} (use {CATEGORIES})")
        quota[cat] = int(n)
    return quota


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", type=Path, default=OFFICIAL, help="SDF (default: official line_track.sdf)")
    ap.add_argument("--out", type=Path, help="output .npz (default experiments/results/perception/rec/<map>.npz)")
    ap.add_argument("--count", type=int, default=40)
    ap.add_argument("--hill-frac", type=float, default=0.3, help="share of frames on/in front of hills")
    ap.add_argument("--quota", help="frames per scene type, e.g. straight=6,corner=6,crossing=5,loop=5,hill=6")
    ap.add_argument("--seed", type=int, help="pose sampling seed (default random, printed and saved)")
    ap.add_argument("--lateral", type=float, default=0.03, help="max lateral offset from the centreline [m]")
    ap.add_argument("--yaw-deg", type=float, default=8.0, help="max heading offset [deg]")
    ap.add_argument("--settle", type=float, default=0.5, help="sim seconds after each teleport")
    ap.add_argument("--min-sep", type=float, default=0.12, help="min arc spacing between poses [m]")
    ap.add_argument("--png-level", type=int, default=9)
    ap.add_argument("--partition", help="GZ_PARTITION (default lf_p_rec_<pid>)")
    ap.add_argument("--domain", type=int, default=60, help="ROS_DOMAIN_ID exported to Gazebo (50-69)")
    a = ap.parse_args()
    out = a.out or ROOT / "experiments/results/perception/rec" / f"{Path(a.world).stem}.npz"
    try:
        record(a.world, out, a.count, a.hill_frac, parse_quota(a.quota), a.seed, a.lateral, a.yaw_deg,
               a.settle, a.min_sep, a.partition, a.domain, a.png_level)
    except (ValueError, RuntimeError, FileNotFoundError) as error:
        ap.error(str(error))


if __name__ == "__main__":
    main()
