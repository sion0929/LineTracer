#!/usr/bin/env python3
"""Replay three perception profiles side by side while a map accumulates.

  python3 experiments/viewer/make_replay_video.py [--maps 0001_official ...]

Each output video covers one map. Panels compare rule / Isaac dilate / Isaac close.
GT ink and TP/FP scoring reuse experiments/eval/perception_map_eval.py.
"""
import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "experiments/eval"))
import perception_map_eval as pme

PROFILES = (
    ("v1_rule", "V1-Near320&Far640 (규칙 기반)"),
    ("v1_dl_r3", "V4-NearB320-Isaac dilate"),
    ("v1_dl_r3c", "V4-NearB320-Isaac close"),
)
DEFAULT_MAPS = ("0001_official", "0002_decoy_0950", "0004_decoy_0952")
RUNS = ROOT / "experiments/results/lead/isaac_replay"
VIDEO_DIR = ROOT / "docs/videos"
IMAGE_DIR = ROOT / "docs/report_img"
FFMPEG = Path("/home/rvlab/miniforge3/envs/lerobot/bin/ffmpeg")

FRAME_DT = 0.2
FPS = 20
RECENT_S = 1.0
FOV_DEG = 62.0
FOV_M = 1.3
PROXIMITY_M = 1.0

BG = "#14181c"
FG = "#e8eaed"
INK = "#9aa1a8"
TP_OLD = "#2f9e5a"
TP_NEW = "#8bff5a"
FP_OLD = "#9c2222"
FP_NEW = "#ff4d4d"
PATH = "#59d0ff"
ROBOT = "#ffffff"
CAM = "#ffd400"


def setup_fonts():
    for path in (ROOT / "experiments/results/lead/fonts").glob("*.tt*"):
        font_manager.fontManager.addfont(str(path))
    families = [f.name for f in font_manager.fontManager.ttflist if "CJK" in f.name]
    plt.rcParams["font.family"] = families[:1] + ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False


def resolve_world(value):
    world = Path(value)
    if not world.exists() and world.parts[:2] == ("/", "ws"):
        world = ROOT.joinpath(*world.parts[2:])
    return world


def load_truth(world):
    sidecar = world.with_suffix(".json")
    meta = json.loads(sidecar.read_text()) if sidecar.is_file() else pme.official_metadata(world)
    ink, centers, is_decoy = pme.ground_truth(meta)
    return ink, centers, float(meta.get("line_width_m", 0.02)), is_decoy


def proximity_time(centers, trajectory, radius=PROXIMITY_M):
    traj_xy = trajectory[:, 1:3]
    if len(traj_xy) < 2:
        return np.full(len(centers), np.inf)
    a = traj_xy[:-1]
    delta = traj_xy[1:] - a
    length2 = np.maximum(np.sum(delta ** 2, axis=1), 1e-20)
    seg_time = trajectory[1:, 0]
    out = np.full(len(centers), np.inf)
    for start in range(0, len(centers), 256):
        points = centers[start:start + 256]
        offsets = points[:, None, :] - a[None, :, :]
        fraction = np.clip(np.einsum("ijk,jk->ij", offsets, delta) / length2, 0, 1)
        residual = offsets - fraction[..., None] * delta
        distance = np.sqrt(np.einsum("ijk,ijk->ij", residual, residual))
        near = distance <= radius + 1e-10
        out[start:start + len(points)] = np.where(
            near, seg_time[None, :], np.inf).min(axis=1)
    return out


def coverage_time(centers, xy_good, t_good, tolerance):
    if not len(xy_good) or not len(centers):
        return np.full(len(centers), np.inf)
    tree = cKDTree(xy_good)
    neighbourhoods = tree.query_ball_point(centers, tolerance + 1e-10, workers=-1)
    out = np.full(len(centers), np.inf)
    for index, hits in enumerate(neighbourhoods):
        if hits:
            out[index] = t_good[hits].min()
    return out


def load_run(profile, map_name, truth):
    run_dir = RUNS / profile / "runs" / map_name
    job = json.loads((run_dir / "job.json").read_text())
    world = resolve_world(job["map"])
    ink, centers, width, is_decoy = truth
    tolerance = width / 2 + 0.01
    with np.load(run_dir / "map_world.npz") as data:
        xy = np.asarray(data["xy"], float).reshape(-1, 2)
        times = np.asarray(data["t"], float)
    trajectory = np.load(run_dir / "trajectory.npy").astype(float)
    good = pme.segment_distance(xy, ink) <= tolerance + 1e-10
    good_xy, good_t = xy[good], times[good]
    return {
        "world": world,
        "xy": xy,
        "t": times,
        "traj": trajectory,
        "good": good,
        "near_time": proximity_time(centers, trajectory),
        "cover_time": coverage_time(centers, good_xy, good_t, tolerance),
        "is_decoy": is_decoy,
        "tolerance": tolerance,
    }


def square_limits(arrays, margin=0.18):
    points = np.concatenate([np.asarray(a, float).reshape(-1, 2) for a in arrays if len(a)])
    low = points.min(axis=0) - margin
    high = points.max(axis=0) + margin
    centre = (low + high) / 2
    half = max(high - low) / 2
    return (centre[0] - half, centre[0] + half), (centre[1] - half, centre[1] + half)


class Panel:
    def __init__(self, ax, ink, width, title, xlim, ylim):
        self.ax = ax
        self.ax.set_facecolor(BG)
        self.ax.set_xlim(*xlim)
        self.ax.set_ylim(*ylim)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        for spine in self.ax.spines.values():
            spine.set_color("#3a4047")

        linewidth = width * 72 * 0.9
        self.ink = LineCollection(ink, colors=INK, linewidths=linewidth, alpha=0.55, zorder=1)
        self.ax.add_collection(self.ink)

        self.fov = Polygon(np.zeros((0, 2)), closed=True, facecolor=CAM, edgecolor=CAM,
                           alpha=0.10, linewidth=0.8, zorder=0.8)
        self.ax.add_patch(self.fov)
        self.path, = self.ax.plot([], [], color=PATH, linewidth=1.1, alpha=0.85, zorder=2)

        common = dict(marker="s", linewidths=0, zorder=3)
        self.tp_old = self.ax.scatter([], [], s=5.0, c=TP_OLD, **common)
        self.tp_new = self.ax.scatter([], [], s=6.5, c=TP_NEW, zorder=4, marker="s", linewidths=0)
        self.fp_old = self.ax.scatter([], [], s=5.0, c=FP_OLD, **common)
        self.fp_new = self.ax.scatter([], [], s=6.5, c=FP_NEW, zorder=4, marker="s", linewidths=0)

        self.robot = self.ax.scatter([], [], s=80, c=ROBOT, edgecolors="#111111",
                                     linewidths=1.2, zorder=6)
        self.arrow = self.ax.quiver([], [], [], [], angles="xy", scale_units="xy", scale=1,
                                    color=CAM, width=0.007, headwidth=4.5, headlength=5.5,
                                    zorder=6)
        self.ax.set_title(title, color=FG, fontsize=14.5, pad=26)
        self.stats = self.ax.text(0.5, 1.015, "", transform=self.ax.transAxes,
                                  ha="center", va="bottom", color=FG, fontsize=12.5)

    def update(self, run, now):
        times, xy, good = run["t"], run["xy"], run["good"]
        included = times <= now
        recent = included & (times > now - RECENT_S)
        true_positive = included & good
        false_positive = included & ~good

        self.tp_old.set_offsets(xy[true_positive & ~recent])
        self.tp_new.set_offsets(xy[true_positive & recent])
        self.fp_old.set_offsets(xy[false_positive & ~recent])
        self.fp_new.set_offsets(xy[false_positive & recent])

        total = int(included.sum())
        hits = int(true_positive.sum())
        precision = hits / total if total else None
        near = run["near_time"] <= now
        covered = run["cover_time"] <= now
        denominator = int(near.sum())
        numerator = int((near & covered).sum())
        recall = numerator / denominator if denominator else None
        precision_text = f"{precision:.1%}" if precision is not None else "—"
        recall_text = f"{recall:.1%}" if recall is not None else "—"
        self.stats.set_text(f"정밀도 {precision_text}   ·   재현율 {recall_text} (1 m)")

        trajectory = run["traj"]
        index = int(np.searchsorted(trajectory[:, 0], now, side="right")) - 1
        index = max(index, 0)
        position = trajectory[index, 1:3]
        heading = 0.0
        previous = index - 1
        while previous >= 0 and np.linalg.norm(position - trajectory[previous, 1:3]) < 1e-4:
            previous -= 1
        if previous >= 0:
            delta = position - trajectory[previous, 1:3]
        elif len(trajectory) > 1:
            delta = trajectory[1, 1:3] - trajectory[0, 1:3]
        else:
            delta = np.zeros(2)
        if np.linalg.norm(delta) > 0:
            heading = float(np.arctan2(delta[1], delta[0]))

        prefix = trajectory[:index + 1, 1:3]
        self.path.set_data(prefix[:, 0], prefix[:, 1])
        angles = np.linspace(heading - np.deg2rad(FOV_DEG / 2),
                             heading + np.deg2rad(FOV_DEG / 2), 24)
        arc = position + FOV_M * np.stack((np.cos(angles), np.sin(angles)), axis=1)
        self.fov.set_xy(np.vstack((position, arc)))
        self.robot.set_offsets([position])
        self.arrow.set_offsets([position])
        self.arrow.set_UVC([0.36 * np.cos(heading)], [0.36 * np.sin(heading)])


def render_map(map_name, truth):
    runs = [load_run(profile, map_name, truth) for profile, _ in PROFILES]
    end_time = max(float(run["traj"][-1, 0]) for run in runs)
    times = np.arange(0.0, end_time + 1e-9, FRAME_DT)
    if times[-1] < end_time - 1e-9:
        times = np.append(times, end_time)

    arrays = [truth[0].reshape(-1, 2), truth[1]]
    for run in runs:
        arrays.extend([run["xy"], run["traj"][:, 1:3]])
    xlim, ylim = square_limits(arrays)

    fig, axes = plt.subplots(1, 3, figsize=(19.2, 7.2), facecolor=BG)
    fig.subplots_adjust(left=0.015, right=0.995, top=0.855, bottom=0.055, wspace=0.055)
    panels = [Panel(ax, truth[0], truth[2], title, xlim, ylim)
              for ax, (_, title) in zip(axes, PROFILES)]
    time_text = fig.text(0.012, 0.962, "", color=FG, fontsize=16, weight="bold")
    fig.text(0.988, 0.962, f"{map_name}  ·  4×", color="#aeb6bd",
             fontsize=12, ha="right")
    fig.text(0.5, 0.018,
             "GT 잉크(회색)   정답 근처(초록)   오답(빨강)   최근 1초 진하게   "
             "로봇·시야 62° / 1.3 m",
             color="#aeb6bd", fontsize=10.5, ha="center")

    VIDEO_DIR.mkdir(parents=True, exist_ok=True)
    IMAGE_DIR.mkdir(parents=True, exist_ok=True)
    video_path = VIDEO_DIR / f"replay_{map_name}_rule_vs_dl.mp4"
    final_path = IMAGE_DIR / f"replay_{map_name}_final.png"
    last_frame = None

    with tempfile.TemporaryDirectory(prefix=f"replay_{map_name}_", dir=VIDEO_DIR) as tmp:
        tmp_dir = Path(tmp)
        for frame_index, now in enumerate(times):
            for panel, run in zip(panels, runs):
                panel.update(run, float(now))
            time_text.set_text(f"sim t = {now:5.1f} s")
            frame_path = tmp_dir / f"frame_{frame_index:05d}.png"
            fig.savefig(frame_path, dpi=100, facecolor=fig.get_facecolor())
            last_frame = frame_path

        shutil.copyfile(last_frame, final_path)
        command = [
            str(FFMPEG), "-y", "-loglevel", "error", "-framerate", str(FPS),
            "-i", str(tmp_dir / "frame_%05d.png"),
            "-vcodec", "libx264", "-pix_fmt", "yuv420p", "-crf", "23",
            str(video_path),
        ]
        subprocess.run(command, check=True)
        frame_count = len(times)

    plt.close(fig)
    return video_path, final_path, frame_count, end_time, runs


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--maps", nargs="+", default=list(DEFAULT_MAPS))
    args = parser.parse_args()

    setup_fonts()
    truth_cache = {}
    for map_name in args.maps:
        run_dir = RUNS / PROFILES[0][0] / "runs" / map_name
        job = json.loads((run_dir / "job.json").read_text())
        world = resolve_world(job["map"])
        if world not in truth_cache:
            truth_cache[world] = load_truth(world)
        video, final, frames, duration, runs = render_map(map_name, truth_cache[world])
        size = video.stat().st_size
        print(f"{map_name}: {video.relative_to(ROOT)} | {size} bytes | "
              f"{duration:.1f} s sim | {frames} frames | {frames / FPS:.2f} s video "
              f"| final {final.relative_to(ROOT)}")
        for (profile, _), run in zip(PROFILES, runs):
            total = len(run["xy"])
            hits = int(run["good"].sum())
            near = np.isfinite(run["near_time"])
            covered = run["cover_time"] <= duration + 1e-9
            denominator = int(near.sum())
            numerator = int((near & covered).sum())
            print(f"  {profile}: precision {hits / total:.4f} ({hits}/{total}) "
                  f"recall {numerator / denominator:.4f} ({numerator}/{denominator})")


if __name__ == "__main__":
    main()
