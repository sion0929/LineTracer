#!/usr/bin/env python3
"""Speed + accuracy of the package perception plugins on recorded frames (dev tool).

Recordings come from experiments/eval/record_perception.py (camera frames + true pose + true
line masks). Every plugin runs through the node's own code path: make_perception(name, node)
and plugin.process(bgr, gray, pitch, x_max), with the node's pitch handling (follower_node.py
pitch_at / on_image) and the frames' true pose as the IMU reading:

  |IMU pitch| > 3 deg (on a slope)  -> pitch = REST_PITCH, x_max = 0.50 m  (node: hill rule)
  otherwise (flat ground)           -> pitch = REST_PITCH (--pitch rest, default) or the true
                                       body pitch (--pitch imu), x_max = vision.X_FAR
node.pitch_f (used by rule_plane) is the true nose-up body pitch. The robot is static, so the
node's rocking test never trips.

  python3 experiments/eval/perception_bench.py REC.npz [REC2.npz ...] --out DIR
  python3 experiments/eval/perception_bench.py REC.npz --random 5 --out DIR   # + 5 sampled maps
  python3 experiments/eval/perception_bench.py --random 3 --suites hills60 --seed 4 --out DIR

Metrics (frames split into flat / hill; hill = robot on a slope or slope inside the view):
  * latency: CPU time of plugin.process() per frame, mean and p95 [ms], one process,
    cv2.setNumThreads(--threads, default 1), first frame excluded as warm-up
  * BEV IoU / precision / recall: plugin line mask vs the true VISIBLE line (gt_bev, projected
    with the true pose into the package BEV grid), both restricted to the plugin's valid mask
  * cover: share of the true visible line cells inside the plugin's valid mask
  * IoU seg: plugin line vs the true IMAGE mask warped with the same vision.bev_warp (pure
    segmentation quality, no geometry error)
  * lateral error [cm] at x = 0.35 / 0.50 / 0.70 m: route position (probe_y) vs the centre of
    the nearest detected run in that BEV row (+-1 row), matched within 10 cm
  * detection rate: share of visible route probes with a detected run within 10 cm

Writes DIR/perception_bench.json and DIR/perception_bench.md.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT / "src/line_follower"
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(ROOT / "experiments/worlds"))

from line_follower import vision  # noqa: E402
from line_follower.perception import make_perception  # noqa: E402

try:
    from line_follower.follower_node import REST_PITCH  # noqa: E402
except ImportError:                                     # no rclpy: same constant, see follower_node.py
    REST_PITCH = math.radians(0.33)

PLUGINS = ("rule", "rule_plane", "rule_hill", "dl")
HILL_PITCH = math.radians(3.0)        # follower_node.pitch_at / on_image
HILL_X_MAX = 0.50
MATCH_GATE = 0.10
GROUP_ORDER = ("ALL", "flat", "hill")


# ------------------------------------------------------------------ recordings
class Recording:
    def __init__(self, path):
        self.path = Path(path)
        d = np.load(self.path)
        self.d = d
        self.meta = json.loads(str(d["meta"]))
        self.n = len(d["poses"])
        self.shape = tuple(int(v) for v in d["bev_shape"])
        grid = self.meta["bev"]
        if (self.shape != (vision.BH, vision.BW) or abs(grid["res"] - vision.RES) > 1e-12
                or abs(grid["x_far"] - vision.X_FAR) > 1e-12 or abs(grid["y_half"] - vision.Y_HALF) > 1e-12):
            raise ValueError(f"{path}: recorded for another BEV grid {grid}; re-record it")
        self.name = self.meta["map"]
        self.probe_x = d["probe_x"]
        self.probe_y = d["probe_y"]
        self.pitch_up = -d["poses"][:, 4]            # Gazebo pitch > 0 is nose down
        self.hill = d["hill"]
        self.category = d["category"]

    def frame(self, i):
        a, b = self.d["frames_off"][i:i + 2]
        return cv2.imdecode(self.d["frames_png"][a:b], cv2.IMREAD_COLOR)

    def image_mask(self, i):
        a, b = self.d["gtmask_off"][i:i + 2]
        return cv2.imdecode(self.d["gtmask_png"][a:b], cv2.IMREAD_UNCHANGED)

    def gt(self, i):
        return np.unpackbits(self.d["gt_bev"][i])[:self.shape[0] * self.shape[1]].reshape(self.shape).astype(bool)


# ------------------------------------------------------------------ node stand-in
class FakeNode:
    """Just enough of FollowerNode for the perception plugins (parameters, pitch_f, pose)."""

    def __init__(self, overrides=None):
        self.params = {}
        self.overrides = dict(overrides or {})
        self.pitch_f = 0.0
        self.pose = (0.0, 0.0, 0.0)

    def declare_parameter(self, name, value=None, descriptor=None):
        self.params[name] = self.overrides.get(name, value)
        return SimpleNamespace(value=self.params[name])

    def get_parameter(self, name):
        return SimpleNamespace(value=self.params[name])

    def get_logger(self):
        return SimpleNamespace(info=print, warn=print, warning=print, error=print, debug=lambda *a: None)


def plugin_overrides(name):
    """Plugin parameters ('<prefix>.<key>') from config/<name>.yaml, as the launch file would pass."""
    path = PKG / "config" / f"{name}.yaml"
    if not path.is_file():
        return {}
    try:
        import yaml
    except ImportError:
        return {}
    data = yaml.safe_load(path.read_text()) or {}
    params = (data.get("follower_node") or {}).get("ros__parameters") or {}
    return {k: v for k, v in params.items() if "." in k}


def make_plugin(name):
    overrides = plugin_overrides(name)
    if name == "dl" and not overrides.get("dl.model_path"):
        overrides["dl.model_path"] = str(ROOT / "experiments/dl/line_seg_v2.onnx")
    node = FakeNode(overrides)
    return node, make_perception(name, node)


def node_pitch(pitch_up, mode):
    """(pitch, x_max) exactly as FollowerNode.on_image would pass them for a static robot."""
    on_slope = abs(pitch_up) > HILL_PITCH
    if on_slope:
        return REST_PITCH, HILL_X_MAX
    return (REST_PITCH if mode == "rest" else pitch_up), vision.X_FAR


# ------------------------------------------------------------------ metrics
def row_runs(row_mask):
    """Centres (BEV column, float) of the runs of True in a 1-D mask."""
    edges = np.flatnonzero(np.diff(np.concatenate(([0], row_mask.astype(np.int8), [0]))))
    return [(a + b - 1) / 2.0 for a, b in zip(edges[::2], edges[1::2])]


def lateral_errors(line, probe_x, probe_y):
    """Per probe: (visible, error_m or None). line = plugin line & valid (bool BEV)."""
    out = []
    for x0, y_gt in zip(probe_x, probe_y):
        if not np.isfinite(y_gt):
            out.append((False, None))
            continue
        r0 = int(round((vision.X_FAR - x0) / vision.RES))
        band = line[max(r0 - 1, 0):r0 + 2].any(0)
        best = None
        for c in row_runs(band):
            err = abs((vision.Y_HALF - c * vision.RES) - y_gt)
            best = err if best is None or err < best else best
        out.append((True, best if best is not None and best <= MATCH_GATE else None))
    return out


class Acc:
    def __init__(self, n_probes):
        self.c = dict(frames=0, inter=0, union=0, npred=0, ngt=0, gt_all=0, gt_cov=0, seg_inter=0, seg_union=0)
        self.errs = [[] for _ in range(n_probes)]
        self.vis = np.zeros(n_probes, int)
        self.hit = np.zeros(n_probes, int)
        self.ms = []
        self.frame_iou = []

    def add(self, counts, lat, ms):
        for k, v in counts.items():
            self.c[k] += int(v)
        self.c["frames"] += 1
        self.frame_iou.append(counts["inter"] / counts["union"] if counts["union"] else np.nan)
        for k, (visible, err) in enumerate(lat):
            if visible:
                self.vis[k] += 1
                if err is not None:
                    self.hit[k] += 1
                    self.errs[k].append(err)
        if ms is not None:
            self.ms.append(ms)

    def result(self, probe_x):
        c = self.c
        all_err = [e for errs in self.errs for e in errs]

        def pct(v, q):
            return round(100 * float(np.percentile(v, q)), 2) if len(v) else None
        r = dict(frames=c["frames"],
                 iou=round(c["inter"] / max(c["union"], 1), 4),
                 precision=round(c["inter"] / max(c["npred"], 1), 4),
                 recall=round(c["inter"] / max(c["ngt"], 1), 4),
                 cover=round(c["gt_cov"] / max(c["gt_all"], 1), 4),
                 iou_seg=round(c["seg_inter"] / max(c["seg_union"], 1), 4),
                 iou_frame_mean=round(float(np.nanmean(self.frame_iou)), 4) if np.isfinite(self.frame_iou).any() else None,
                 lateral_cm={f"{x:.2f}": dict(p50=pct(e, 50), p95=pct(e, 95), mean=round(100 * float(np.mean(e)), 2)
                                              if e else None, n=len(e)) for x, e in zip(probe_x, self.errs)},
                 lateral_all_cm=dict(p50=pct(all_err, 50), p95=pct(all_err, 95), n=len(all_err)),
                 detection={f"{x:.2f}": round(h / v, 4) if v else None for x, h, v in zip(probe_x, self.hit, self.vis)},
                 detection_rate=round(self.hit.sum() / max(self.vis.sum(), 1), 4),
                 probes_visible=int(self.vis.sum()), probes_detected=int(self.hit.sum()))
        if self.ms:
            r.update(ms_mean=round(float(np.mean(self.ms)), 3), ms_p95=round(float(np.percentile(self.ms, 95)), 3),
                     ms_max=round(float(np.max(self.ms)), 3), timed_frames=len(self.ms))
        return r


def frame_counts(out, gt, img_mask, pitch, x_max):
    line, valid = out.line > 0, out.valid > 0
    pred, truth = line & valid, gt & valid
    warped, _ = vision.bev_warp((img_mask == 1).astype(np.uint8) * 255, pitch, x_max)
    seg = (warped > 127) & valid
    return dict(inter=(pred & truth).sum(), union=(pred | truth).sum(), npred=pred.sum(), ngt=truth.sum(),
                gt_all=gt.sum(), gt_cov=truth.sum(), seg_inter=(pred & seg).sum(), seg_union=(pred | seg).sum()), pred


def evaluate_plugin(name, recordings, pitch_mode):
    try:
        node, plugin = make_plugin(name)
    except Exception as error:                      # dl is a stub: never fake a result
        return dict(status="skipped", reason=f"{type(error).__name__}: {error}")
    n_probes = len(recordings[0].probe_x)
    probe_x = [float(x) for x in recordings[0].probe_x]
    groups = {}
    first = True
    for rec in recordings:
        for i in range(rec.n):
            bgr = rec.frame(i)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            node.pitch_f = float(rec.pitch_up[i])
            pitch, x_max = node_pitch(rec.pitch_up[i], pitch_mode)
            if first:                               # warm-up (imports, allocations)
                plugin.process(bgr, gray, pitch, x_max)
                first = False
            t0 = time.perf_counter()
            out = plugin.process(bgr, gray, pitch, x_max)
            ms = 1000.0 * (time.perf_counter() - t0)
            counts, pred = frame_counts(out, rec.gt(i), rec.image_mask(i), pitch, x_max)
            lat = lateral_errors(pred, rec.probe_x, rec.probe_y[i])
            keys = ("ALL", "hill" if rec.hill[i] else "flat", f"map:{rec.name}", f"type:{rec.category[i]}")
            for key in keys:
                groups.setdefault(key, Acc(n_probes)).add(counts, lat, ms)
    return dict(status="ok", groups={k: a.result(probe_x) for k, a in groups.items()})


# ------------------------------------------------------------------ random maps
def record_random(n, suites, seed, out_dir, count, jobs, hill_frac):
    import sample_maps

    sample_dir, seed, picked = sample_maps.run(n, suites, seed, out=out_dir / "maps")
    rec_dir = out_dir / "rec"
    rec_dir.mkdir(parents=True, exist_ok=True)
    batch = f"{os.getpid()}"

    def one(k_item):
        k, (suite, row) = k_item
        world = ROOT / row["path"]
        out = rec_dir / f"{world.stem}.npz"
        if out.is_file():
            return out
        cmd = [sys.executable, str(ROOT / "experiments/eval/record_perception.py"), "--world", str(world),
               "--out", str(out), "--count", str(count), "--hill-frac", str(hill_frac), "--seed", str(seed + k),
               "--partition", f"lf_p_pb_{batch}_{k}", "--domain", str(50 + k % 20)]
        with (rec_dir / f"{world.stem}.log").open("w") as log:
            code = subprocess.call(cmd, stdout=log, stderr=subprocess.STDOUT)
        if code or not out.is_file():
            print(f"recording failed for {world.name}, see {rec_dir / (world.stem + '.log')}", flush=True)
            return None
        print(f"recorded {out.name}", flush=True)
        return out

    with ThreadPoolExecutor(max_workers=max(1, min(jobs, 3))) as pool:
        paths = list(pool.map(one, enumerate(picked)))
    return seed, sample_dir, [p for p in paths if p]


# ------------------------------------------------------------------ report
def fmt(v, nd=3):
    return "-" if v is None else (f"{v:.{nd}f}" if isinstance(v, float) else str(v))


def lat_cell(r, x):
    e = r["lateral_cm"].get(x) or {}
    return f"{fmt(e.get('p50'), 1)} / {fmt(e.get('p95'), 1)}"


def markdown(report):
    probes = [f"{x:.2f}" for x in report["probe_x"]]
    recs = ", ".join("%s (%d frames, %d hill)" % (r["map"], r["frames"], r["hill_frames"]) for r in report["recordings"])
    lines = [f"# Perception bench ({report['created']})", "",
             f"- recordings: {recs}",
             f"- pitch mode `{report['pitch_mode']}` (REST_PITCH {math.degrees(REST_PITCH):.2f} deg; on a slope REST_PITCH "
             f"and x_max {HILL_X_MAX} m as in the node), OpenCV threads {report['threads']}, latency = plugin.process() only",
             "- IoU / P / R: BEV line vs true visible line inside the plugin's valid mask; cover: true line cells inside "
             "valid; IoU seg: vs the true image mask warped with the same bev_warp",
             f"- lateral error p50 / p95 [cm] at x = {' / '.join(probes)} m; det: route probes with a detected run "
             f"within {MATCH_GATE * 100:.0f} cm", ""]
    if report.get("random"):
        rnd = report["random"]
        lines += [f"- random maps: seed {rnd['seed']} ({rnd['suites']}), sample in `{rnd['sample_dir']}`", ""]
    head = ("| plugin | split | frames | ms mean | ms p95 | IoU | P | R | cover | IoU seg | "
            + " | ".join(f"lat {x} p50/p95" for x in probes) + " | lat all p95 | det |")
    lines += ["## Summary", "", head, "|" + "---|" * (head.count("|") - 1)]
    for name, res in report["plugins"].items():
        if res["status"] != "ok":
            lines.append(f"| {name} | skipped: {res['reason'][:90]} |" + " |" * (head.count("|") - 3))
            continue
        for split in GROUP_ORDER:
            r = res["groups"].get(split)
            if not r:
                continue
            lines.append(f"| {name} | {split} | {r['frames']} | {fmt(r.get('ms_mean'), 2)} | {fmt(r.get('ms_p95'), 2)} | "
                         f"{r['iou']:.3f} | {r['precision']:.3f} | {r['recall']:.3f} | {r['cover']:.3f} | "
                         f"{r['iou_seg']:.3f} | " + " | ".join(lat_cell(r, x) for x in probes)
                         + f" | {fmt(r['lateral_all_cm']['p95'], 1)} | {100 * r['detection_rate']:.1f}% |")
    ok = {n: r for n, r in report["plugins"].items() if r["status"] == "ok"}
    for prefix, title in (("map:", "Per map (ALL frames)"), ("type:", "Per scene type (ALL maps)")):
        keys = sorted({k for r in ok.values() for k in r["groups"] if k.startswith(prefix)})
        if not keys:
            continue
        lines += ["", f"## {title}", "", "| " + prefix[:-1] + " | frames | "
                  + " | ".join(f"{n} IoU / lat p95 / det" for n in ok) + " |",
                  "|---|---|" + "---|" * len(ok)]
        for key in keys:
            cells = []
            frames = None
            for r in ok.values():
                g = r["groups"].get(key)
                frames = g["frames"] if g else frames
                cells.append("-" if not g else f"{g['iou']:.3f} / {fmt(g['lateral_all_cm']['p95'], 1)} / "
                             f"{100 * g['detection_rate']:.0f}%")
            lines.append(f"| {key[len(prefix):]} | {frames} | " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recordings", nargs="*", type=Path, help="record_perception.py .npz files")
    ap.add_argument("--plugins", default=",".join(PLUGINS))
    ap.add_argument("--pitch", choices=("rest", "imu"), default="rest",
                    help="flat-ground pitch: REST_PITCH (default) or the true body pitch (ideal IMU)")
    ap.add_argument("--threads", type=int, default=1, help="cv2.setNumThreads")
    ap.add_argument("--random", type=int, default=0, help="also sample N maps, record and evaluate them")
    ap.add_argument("--suites", default="general200,official_like50,hills60,stress50")
    ap.add_argument("--seed", type=int, help="map sampling seed for --random (default random, saved)")
    ap.add_argument("--count", type=int, default=40, help="frames per random map")
    ap.add_argument("--hill-frac", type=float, default=0.3)
    ap.add_argument("--jobs", type=int, default=3, help="parallel Gazebo recordings (max 3)")
    ap.add_argument("--out", type=Path, help="output dir (default experiments/results/perception/<time>)")
    a = ap.parse_args()
    out = (a.out or ROOT / "experiments/results/perception" / time.strftime("%Y%m%d-%H%M%S")).resolve()
    out.mkdir(parents=True, exist_ok=True)
    paths = list(a.recordings)
    random_info = None
    if a.random:
        seed, sample_dir, new = record_random(a.random, a.suites.split(","), a.seed, out, a.count, a.jobs, a.hill_frac)
        random_info = dict(seed=seed, suites=a.suites, sample_dir=str(sample_dir.relative_to(ROOT)),
                           recordings=[str(p.relative_to(ROOT)) for p in new])
        paths += new
    if not paths:
        ap.error("no recordings (give .npz files and/or --random N)")
    recordings = [Recording(p) for p in paths]
    cv2.setNumThreads(a.threads)
    report = dict(created=time.strftime("%Y-%m-%d %H:%M:%S"), pitch_mode=a.pitch, threads=a.threads,
                  rest_pitch_deg=round(math.degrees(REST_PITCH), 3), match_gate_m=MATCH_GATE,
                  probe_x=[float(x) for x in recordings[0].probe_x], random=random_info,
                  recordings=[dict(path=str(r.path.resolve().relative_to(ROOT)) if r.path.resolve().is_relative_to(ROOT)
                                   else str(r.path), map=r.name, frames=r.n, hill_frames=int(r.hill.sum()),
                                   generator=r.meta.get("generator"), capture_seed=r.meta.get("capture_seed"))
                              for r in recordings],
                  plugins={})
    for name in [p for p in a.plugins.split(",") if p]:
        t0 = time.time()
        report["plugins"][name] = evaluate_plugin(name, recordings, a.pitch)
        res = report["plugins"][name]
        if res["status"] == "ok":
            g = res["groups"]["ALL"]
            print(f"{name:11} IoU {g['iou']:.3f}  lat p95 {fmt(g['lateral_all_cm']['p95'], 1)} cm  "
                  f"det {100 * g['detection_rate']:.1f}%  {g['ms_mean']:.2f}/{g['ms_p95']:.2f} ms  "
                  f"({time.time() - t0:.0f} s)", flush=True)
        else:
            print(f"{name:11} skipped: {res['reason']}", flush=True)
    (out / "perception_bench.json").write_text(json.dumps(report, indent=1) + "\n")
    (out / "perception_bench.md").write_text(markdown(report))
    print(f"wrote {out}/perception_bench.json, perception_bench.md")


if __name__ == "__main__":
    main()
