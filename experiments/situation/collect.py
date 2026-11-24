#!/usr/bin/env python3
"""DEV ONLY: build the situation datasets (Gazebo runs + synthetic replay), labelled from ground truth.

Map split (all seeds >= 1000; the 6 maps of random_drive --seed 1457145442 are excluded everywhere,
seeds 0-39 and the official track are never used here):
  gz_train   30 suite maps driven in Gazebo for training
  gz_val     12 other suite maps driven in Gazebo: the held-out real data for model selection/metrics
  synth      every other suite map (and the gz_train maps), synthetic replay only

  python3 experiments/situation/collect.py maps                 # split + spawn-shifted worlds
  python3 experiments/situation/collect.py gazebo gz_train      # bench runs with collect_node.py
  python3 experiments/situation/collect.py gazebo gz_val
  python3 experiments/situation/collect.py synth --workers 6    # synthetic replay -> data/synth.npz
  python3 experiments/situation/collect.py build                # Gazebo runs -> data/gz_train.npz, gz_val.npz

Each Gazebo map is driven in segments: from the real start, and from spawn points at ~30 % / ~55 %
of the route and ~4 m before the end (the rule follower often fails at the first sharp corner, so
spawning further on covers the whole map). A spawn world is the map's SDF with the robot pose moved
to a straight, level route point; its JSON has the centre line cut at that point (bench scoring)
and points back to the full map (labels).
"""
import argparse
import csv
import json
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "experiments/worlds"))
sys.path.insert(0, str(ROOT / "src/line_follower"))
from label import Track  # noqa: E402
from line_follower.situation import INDEX, SITUATIONS  # noqa: E402
from line_follower.situation import features as FE  # noqa: E402

DATA = HERE / "data"
SPAWN = DATA / "maps" / "spawn"
EXCLUDED = {1055, 1059, 5036, 7018, 7037, 9024}          # random_drive --seed 1457145442 maps
SUITES = ("general200", "official_like50", "stress50", "hills60")
GZ_TRAIN = {"general200": range(1000, 1012), "official_like50": range(5000, 5008),
            "stress50": range(7000, 7006), "hills60": range(9000, 9004)}
GZ_VAL = {"general200": range(1100, 1104), "official_like50": range(5040, 5044),
          "stress50": range(7040, 7042), "hills60": range(9050, 9052)}
END_CODE = {0: "open", 1: "t", 2: "len"}


def suite_rows():
    import sample_maps
    out = []
    for suite in SUITES:
        for row in sample_maps.load_index(suite):
            if int(row["seed"]) not in EXCLUDED:
                out.append((suite, row))
    return out


def in_split(split, suite, seed):
    return seed in split.get(suite, ())


def ensure(suite, row):
    import sample_maps
    sample_maps.ensure_map(suite, row)
    return ROOT / row["path"]


def spawn_points(track):
    """Arc positions on straight, level route stretches near 30 %, 55 % and L - 4 m."""
    good = []
    ev = [s for s, _, _, _ in track.events] + [a for a, b, _, _ in track.loops] + [b for a, b, _, _ in track.loops]
    for s in np.arange(0.5, track.length - 1.5, 0.05):
        h = np.interp([s - 0.3, s, s + 0.4], track.arc, track.head)
        if np.ptp(h) > math.radians(3):
            continue
        p = np.array([np.interp(s, track.arc, track.xy[:, 0]), np.interp(s, track.arc, track.xy[:, 1])])
        if any(track.on_ramp(p + d * np.array([math.cos(h[1]), math.sin(h[1])])) for d in (-0.4, 0.0, 0.5)):
            continue
        if any(abs(s - e) < 0.4 for e in ev) or any(g0 - 0.4 < s < g1 + 0.4 for g0, g1 in track.gaps):
            continue
        good.append(float(s))
    out = []
    for target in (0.30 * track.length, 0.55 * track.length, track.length - 4.0):
        if good:
            s = min(good, key=lambda g: abs(g - target))
            if all(abs(s - o) > 1.0 for o in out):
                out.append(s)
    return out


def make_spawn(sdf, s):
    """Spawn world + JSON for a map with the robot placed at route arc s."""
    meta = json.loads(sdf.with_suffix(".json").read_text())
    track = Track(meta)
    k = int(np.searchsorted(track.arc, s))
    yaw = float(track.seg_head[min(k, len(track.seg_head) - 1)])
    x, y = track.xy[k]
    name = f"{sdf.stem}_s{int(round(s * 100)):04d}"
    out = SPAWN / f"{name}.sdf"
    SPAWN.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(sdf)
    world = tree.getroot().find("world")
    for inc in world.findall("include"):
        if inc.find("uri") is not None and "turtlebot3" in inc.find("uri").text:
            inc.find("pose").text = f"{x:.9g} {y:.9g} 0.01 0 0 {math.remainder(yaw, math.tau):.9g}"
    for uri in world.iter("uri"):
        if uri.text.startswith("./"):
            uri.text = str((sdf.parent / uri.text[2:]).resolve())
    tree.write(out, encoding="utf-8", xml_declaration=True)
    cut = meta["centerline"][k:]
    arc = [0.0]
    for a, b in zip(cut, cut[1:]):
        arc.append(arc[-1] + math.dist(a[:2], b[:2]))
    meta = dict(meta, centerline=cut, arc_m=arc, start_pose=[x, y, 0.01, 0, 0, yaw],
                situation_source=str(sdf.with_suffix(".json").resolve()), s_offset=float(track.arc[k]))
    out.with_suffix(".json").write_text(json.dumps(meta, separators=(",", ":")) + "\n")
    return out


def cmd_maps(a):
    rows = suite_rows()
    lists = {"gz_train": [], "gz_val": [], "synth": []}
    for suite, row in rows:
        seed = int(row["seed"])
        if in_split(GZ_VAL, suite, seed):
            sdf = ensure(suite, row)
            lists["gz_val"].append(sdf)
        elif in_split(GZ_TRAIN, suite, seed):
            sdf = ensure(suite, row)
            lists["gz_train"].append(sdf)
            lists["synth"].append(sdf)
        else:
            lists["synth"].append(ROOT / row["path"])
    DATA.mkdir(parents=True, exist_ok=True)
    for split in ("gz_train", "gz_val"):
        worlds = []
        for sdf in lists[split]:
            worlds.append(sdf)
            for s in spawn_points(Track(sdf.with_suffix(".json"))):
                worlds.append(make_spawn(sdf, s))
        (DATA / f"{split}_worlds.txt").write_text("\n".join(str(w.relative_to(ROOT)) for w in worlds) + "\n")
        print(split, len(lists[split]), "maps", len(worlds), "worlds")
    with (DATA / "synth_maps.csv").open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["suite", "seed", "path"])
        for suite, row in rows:
            if ROOT / row["path"] in lists["synth"]:
                w.writerow([suite, row["seed"], row["path"]])
    print("synth", len(lists["synth"]), "maps")


def cmd_gazebo(a):
    worlds = [ROOT / p for p in (DATA / f"{a.split}_worlds.txt").read_text().split()]
    if a.limit_worlds:
        worlds = worlds[:a.limit_worlds]
    out = DATA / "gz" / a.split
    if out.exists():
        out = DATA / "gz" / f"{a.split}_{len(list((DATA / 'gz').glob(a.split + '*')))}"
    env = dict(os.environ, LF_PARAMS=str(ROOT / a.params), BENCH_DOMAIN_BASE="50", LAB_NAME="lf_l",
               GZ_PARTITION=f"lf_l_col_{a.split}", ROS_DOMAIN_ID=os.environ.get("ROS_DOMAIN_ID", "22"))
    cmd = [sys.executable, str(ROOT / "experiments/bench.py"), *map(str, worlds), "--follower",
           str(HERE / "collect_node.py"), "--jobs", str(a.jobs), "--off", "0.2", "--limit", str(a.limit),
           "--stall", "15", "--output", str(out)]
    print(" ".join(cmd[:3]), "...", len(worlds), "worlds ->", out, flush=True)
    return subprocess.call(cmd, env=env)


def _single_thread():
    os.environ.update(OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")
    import cv2
    cv2.setNumThreads(1)


def _synth_one(args):
    import synth
    path, seed, variants = args
    try:
        return synth.simulate_map(str(ROOT / Path(path).with_suffix(".json")), seed, variants)
    except Exception as error:  # a broken map must not kill the whole batch
        print("synth failed", path, error, flush=True)
        return []


def cmd_synth(a):
    import sample_maps
    rows = list(csv.DictReader((DATA / "synth_maps.csv").open()))
    if a.max_maps:
        rows = rows[:a.max_maps]
    for row in rows:
        sample_maps.ensure_map(row["suite"], dict(next(r for r in sample_maps.load_index(row["suite"])
                                                   if r["seed"] == row["seed"])))
    variants = tuple(a.variants.split(","))
    jobs = [(r["path"], 1000003 * int(r["seed"]) + 17 + 7 * len(a.out), variants) for r in rows]
    samples = []
    with ProcessPoolExecutor(a.workers, initializer=_single_thread) as pool:
        for k, res in enumerate(pool.map(_synth_one, jobs, chunksize=1)):
            samples += res
            if k % 20 == 0:
                print(f"{k + 1}/{len(jobs)} maps, {len(samples)} samples", flush=True)
    save(DATA / f"{a.out}.npz", samples)


def save(path, samples):
    np.savez_compressed(
        path, X=np.stack([r["f"] for r in samples]).astype(np.float32),
        y=np.array([r["y"] for r in samples], np.int16), tcand=np.array([r["tcand"] for r in samples], np.int8),
        s=np.array([r["s"] for r in samples], np.float32), map=np.array([r["map"] for r in samples]),
        run=np.array([r["run"] for r in samples], np.int16),
        cand_finish=np.array([r["cand_finish"] for r in samples], np.float32),
        features=np.array(FE.FEATURE_NAMES), classes=np.array(SITUATIONS))
    counts = np.bincount([r["y"] for r in samples], minlength=len(SITUATIONS))
    print(path.name, len(samples), "samples", dict(zip(SITUATIONS, counts.tolist())))


def run_samples(run_dir, keep_patches=False):
    """Features + labels of one Gazebo collection run folder."""
    z = np.load(run_dir / "situation.npz")
    job = json.loads((run_dir / "job.json").read_text())
    meta = json.loads(Path(job["map"]).with_suffix(".json").read_text())
    src = meta.get("situation_source") or str(Path(job["map"]).with_suffix(".json"))
    track = Track(src)
    out = []
    for i in range(len(z["t"])):
        if z["done"][i]:
            continue
        patch = np.unpackbits(z["patch"][i])[:FE.PH * FE.PW].reshape(FE.PH, FE.PW).astype(bool)
        n = int(z["n"][i])
        rp = z["rp"][i][:n].astype(float) if n >= 2 else None
        end = END_CODE.get(int(z["end"][i]), "none")
        f = FE.extract_local(patch, rp, end, float(z["pitch"][i]), z["start"][i].astype(float), float(z["x_far"][i]))
        gx, gy, gyaw = z["gt"][i]
        cand = None
        if end == "t" and rp is not None:
            cx, cy = rp[-1]
            cand = (gx + cx * math.cos(gyaw) - cy * math.sin(gyaw), gy + cx * math.sin(gyaw) + cy * math.cos(gyaw))
        lab = track.label((gx, gy, gyaw), cand)
        s_route, _ = track.route_pos((gx, gy, gyaw))
        rec = dict(f=f, y=INDEX[lab], tcand=int(z["tcand"][i]), s=-1.0 if s_route is None else s_route,
                   map=Path(src).stem, run=0,
                   cand_finish=-1.0 if cand is None else float(np.hypot(cand[0] - track.finish[0], cand[1] - track.finish[1])))
        if keep_patches:
            rec.update(patch=z["patch"][i], rp=z["rp"][i], n=n, end=int(z["end"][i]), pitch=float(z["pitch"][i]),
                       x_far=float(z["x_far"][i]), start=z["start"][i])
        out.append(rec)
    return out


def cmd_build(a):
    for split in ("gz_train", "gz_val"):
        samples = []
        for base in sorted((DATA / "gz").glob(f"{split}*")):
            for run in sorted(base.iterdir()):
                if (run / "situation.npz").is_file():
                    rs = run_samples(run)
                    for r in rs:
                        r["run"] = hash(run.name) % 30000
                    samples += rs
        if samples:
            save(DATA / f"{split}.npz", samples)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("maps")
    g = sub.add_parser("gazebo")
    g.add_argument("split", choices=("gz_train", "gz_val"))
    g.add_argument("--params", default="src/line_follower/config/rule.yaml")
    g.add_argument("--jobs", type=int, default=3)
    g.add_argument("--limit", type=float, default=150)
    g.add_argument("--limit-worlds", type=int, default=0)
    s = sub.add_parser("synth")
    s.add_argument("--workers", type=int, default=6)
    s.add_argument("--max-maps", type=int, default=0)
    s.add_argument("--variants", default="forward,forward2,reverse_start",
                   help="synth.simulate_map variants (forward, forward2, reverse_start, markings)")
    s.add_argument("--out", default="synth", help="output name under data/")
    sub.add_parser("build")
    a = ap.parse_args()
    if a.cmd == "gazebo" and not 1 <= a.jobs <= 3:
        ap.error("at most 3 Gazebo runs at once")
    {"maps": cmd_maps, "gazebo": cmd_gazebo, "synth": cmd_synth, "build": cmd_build}[a.cmd](a)


if __name__ == "__main__":
    main()
