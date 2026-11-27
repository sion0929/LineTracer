#!/usr/bin/env python3
"""Side-by-side video of closed-loop Isaac drives: the camera on top, the line map building up below.

    python3 experiments/viewer/make_drive_video.py RUN_DIR [RUN_DIR ...] --names A B --out video.mp4

Each RUN_DIR comes from run_isaac_drive.sh --map-dump ... --video-every K and holds camera.mp4,
map_world.npz (cells in world frame with first-seen sim time t), trajectory.npy
([sim_s, x, y, s, lateral_error]), result.json and job.json. Map cells are coloured against the
true ink (within line width / 2 + 1 cm = correct). Output is H.264 via the lerobot env's ffmpeg.
"""
import argparse
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'experiments/eval'))
import perception_map_eval as pme  # noqa: E402

FFMPEG = '/home/rvlab/miniforge3/envs/lerobot/bin/ffmpeg'
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
HZ = 30.0
PANEL_W, CAM_H, MAP_H = 640, 480, 640
INK, GOOD, BAD, NEW_G, NEW_B, PATH, ROBOT = (200, 200, 200), (70, 165, 70), (60, 60, 215), (40, 230, 40), (40, 40, 255), (30, 140, 245), (20, 20, 20)


def world_of(run):
    job = json.loads((run / 'job.json').read_text())
    w = Path(job['map'])
    if not w.exists() and w.parts[:2] == ('/', 'ws'):
        w = ROOT.joinpath(*w.parts[2:])
    return w


def truth(world):
    side = world.with_suffix('.json')
    meta = json.loads(side.read_text()) if side.is_file() else pme.official_metadata(world)
    ink, centers, _ = pme.ground_truth(meta)
    return ink, centers, float(meta.get('line_width_m', 0.02))


class Run:
    def __init__(self, path, name):
        self.path, self.name = Path(path), name
        z = np.load(self.path / 'map_world.npz')
        self.xy, self.t = z['xy'], z['t']
        self.traj = np.load(self.path / 'trajectory.npy')
        self.res = json.loads((self.path / 'result.json').read_text())
        world = world_of(self.path)
        ink, self.centers, width = truth(world)
        tol = width / 2 + 0.01
        self.good = pme.segment_distance(self.xy, ink) <= tol + 1e-10
        cap = cv2.VideoCapture(str(self.path / 'camera.mp4'))
        self.frames = []
        while True:
            ok, f = cap.read()
            if not ok:
                break
            self.frames.append(cv2.resize(f, (PANEL_W, CAM_H)))
        k = self.res.get('args', [])
        self.every = int(k[k.index('--video-every') + 1]) if '--video-every' in k else 3
        self.t_end = float(self.traj[-1, 0])

    def cam(self, t):
        if not self.frames:
            return np.full((CAM_H, PANEL_W, 3), 30, np.uint8)
        i = min(len(self.frames) - 1, int(t * HZ / self.every))
        return self.frames[i]

    def pose(self, t):
        i = min(len(self.traj) - 1, int(np.searchsorted(self.traj[:, 0], t)))
        j = max(0, i - 3)
        x, y = self.traj[i, 1], self.traj[i, 2]
        h = math.atan2(y - self.traj[j, 2], x - self.traj[j, 1]) if i > j else 0.0
        return x, y, h, i


def frame_for(run, t, lim, fnt, fnt_b):
    x0, x1, y0, y1 = lim
    sc = min((PANEL_W - 40) / (x1 - x0), (MAP_H - 70) / (y1 - y0))
    ox, oy = 20 - x0 * sc, MAP_H - 20 + y0 * sc

    def px(p):
        return np.stack([ox + p[..., 0] * sc, oy - p[..., 1] * sc], -1).astype(np.int32)

    m = np.full((MAP_H, PANEL_W, 3), 250, np.uint8)
    for c in px(run.centers):
        cv2.circle(m, tuple(int(v) for v in c), 2, INK, -1)
    seen = run.t <= t
    new = seen & (run.t > t - 1.0)
    pts = px(run.xy)
    for mask, col in ((seen & run.good & ~new, GOOD), (seen & ~run.good & ~new, BAD),
                      (new & run.good, NEW_G), (new & ~run.good, NEW_B)):
        for q in pts[mask]:
            m[max(0, min(MAP_H - 1, q[1])), max(0, min(PANEL_W - 1, q[0]))] = col
    m = cv2.dilate(m.copy(), np.ones((1, 1), np.uint8))
    x, y, h, i = run.pose(t)
    tr = px(run.traj[:i + 1, 1:3])
    if len(tr) > 1:
        cv2.polylines(m, [tr.reshape(-1, 1, 2)], False, PATH, 2)
    c = px(np.array([x, y]))
    fov = [c]
    for a in np.linspace(-0.543, 0.543, 9):
        fov.append(px(np.array([x + 1.0 * math.cos(h + a), y + 1.0 * math.sin(h + a)])))
    ov = m.copy()
    cv2.fillPoly(ov, [np.array(fov).reshape(-1, 1, 2)], (180, 215, 245))
    m = cv2.addWeighted(ov, 0.35, m, 0.65, 0)
    cv2.circle(m, tuple(int(v) for v in c), 7, ROBOT, -1)
    tip = px(np.array([x + 0.12 * math.cos(h), y + 0.12 * math.sin(h)]))
    cv2.line(m, tuple(int(v) for v in c), tuple(int(v) for v in tip), (255, 255, 255), 2)
    n_seen = int(seen.sum())
    prec = float((seen & run.good).sum()) / max(1, n_seen)
    lat = abs(float(run.traj[i, 4])) * 100

    panel = np.vstack([run.cam(t), m])
    im = Image.fromarray(panel[:, :, ::-1])
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, PANEL_W, 34], fill=(255, 255, 255))
    d.text((10, 4), run.name, fill=(0, 0, 0), font=fnt_b)
    d.rectangle([0, CAM_H, PANEL_W, CAM_H + 34], fill=(255, 255, 255))
    d.text((10, CAM_H + 6), f'지도 정밀도 {prec * 100:.1f}%   칸 {n_seen:,}   횡오차 {lat:.1f} cm', fill=(0, 0, 0), font=fnt)
    if t >= run.t_end - 1e-6:
        r = run.res
        msg = '완주' if r.get('complete') else f"정지: {r.get('reason', '')}"
        d.text((10, CAM_H + MAP_H - 30), f"{msg} · 진행 {r.get('progress_pct', 0):.0f}%", fill=(200, 0, 0), font=fnt_b)
    return np.array(im)[:, :, ::-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--names', nargs='+')
    ap.add_argument('--title', default='')
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--step', type=float, default=0.2, help='sim seconds per video frame')
    ap.add_argument('--fps', type=int, default=20)
    a = ap.parse_args()
    names = a.names or [Path(r).name for r in a.runs]
    runs = [Run(r, n) for r, n in zip(a.runs, names)]
    allp = np.vstack([runs[0].centers] + [r.traj[:, 1:3] for r in runs])
    lim = (allp[:, 0].min() - 0.15, allp[:, 0].max() + 0.15, allp[:, 1].min() - 0.15, allp[:, 1].max() + 0.15)
    fnt, fnt_b = ImageFont.truetype(FONT, 18), ImageFont.truetype(FONT, 21)
    t_end = max(r.t_end for r in runs)
    with tempfile.TemporaryDirectory() as tmp:
        n = 0
        for t in np.arange(0, t_end + 2.0, a.step):
            panels = [frame_for(r, min(t, r.t_end), lim, fnt, fnt_b) for r in runs]
            gap = np.full((panels[0].shape[0], 12, 3), 255, np.uint8)
            row = panels[0]
            for p in panels[1:]:
                row = np.hstack([row, gap, p])
            head = Image.new('RGB', (row.shape[1], 70), 'white')
            hd = ImageDraw.Draw(head)
            hd.text((12, 6), f"{a.title}   ·   sim {t:5.1f} s  ({a.step * a.fps:.0f}배속)", fill=(0, 0, 0), font=fnt_b)
            hd.text((12, 40), "지도: 초록 맞게 본 선 · 빨강 잘못 본 선 · 밝은색 최근 1초 · 주황 지나온 길 · 회색 정답 잉크 · 부채꼴 카메라 시야", fill=(90, 90, 90), font=fnt)
            img = np.vstack([np.array(head)[:, :, ::-1], row])
            if img.shape[1] % 2:
                img = img[:, :-1]
            if img.shape[0] % 2:
                img = img[:-1]
            cv2.imwrite(f'{tmp}/f_{n:05d}.png', img)
            n += 1
        a.out.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run([FFMPEG, '-y', '-loglevel', 'error', '-framerate', str(a.fps), '-i', f'{tmp}/f_%05d.png',
                        '-vcodec', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '23', str(a.out)], check=True)
        cv2.imwrite(str(a.out.with_suffix('.png')), img)
    print(a.out, n, 'frames')


if __name__ == '__main__':
    main()
