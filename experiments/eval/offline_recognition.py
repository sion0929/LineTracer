#!/usr/bin/env python3
"""Offline recognition comparison on Isaac frames (no driving): each perception method's BEV line
mask vs the ground-truth BEV mask of the frame.

    python3 experiments/eval/offline_recognition.py --maps decoy_0978 ... --frames 10 --out DIR

Frames are taken only where the camera is level and at its nominal height (flat ground), because
the rule plugins project onto a flat floor. Scores are pooled over valid BEV cells:
precision / recall of line cells with a 1-cell (5 mm) tolerance, F0.5, and per-map precision for
paired comparison. Writes results.json, results.md and one comparison image per sampled frame.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/line_follower'))
from line_follower import vision  # noqa: E402
from line_follower.perception import make_perception  # noqa: E402

DATA = Path('/home/rvlab/isaac_datasets/26_robotprogramming/decoy1000/test')
MODELS = ROOT / 'experiments/isaac/models'


class _P:
    def __init__(self, v):
        self.value = v


class FakeNode:
    """Just enough of rclpy.Node for perception plugins."""

    def __init__(self, params):
        self.p = dict(params)

    def declare_parameter(self, name, default=None, *a, **k):
        self.p.setdefault(name, default)
        return _P(self.p[name])

    def get_parameter(self, name):
        return _P(self.p[name])

    def has_parameter(self, name):
        return name in self.p

    def get_logger(self):
        class L:
            def info(self, *a, **k): pass
            def warning(self, *a, **k): pass
        return L()


def methods():
    dl = lambda m, splat='close': {'depth.model_path': str(MODELS / f'{m}.onnx'), 'depth.splat': splat,
                                   'depth.use_pitch': False}
    return [
        ('V0 규칙 640', 'rule', {}),
        ('V1 규칙 320', 'rule', {'near_scale': 0.5}),
        ('V0 + 폭 필터 5 cm', 'rule', {'rule.max_width_m': 0.05}),
        ('DL V4 close', 'depth', dl('V4-NearB320-Isaac')),
        ('DL V5 close', 'depth', dl('V5-NearB320-Isaac')),
        ('DL V6 close (오검출 억제)', 'depth', dl('V6-NearB320FP-Isaac')),
        ('혼합 H1 (규칙 ∩ V4)', 'hybrid', {**dl('V4-NearB320-Isaac'), 'hybrid.mode': 'rule_gated'}),
    ]


def level(m):
    fwd = -np.asarray(m)[:3, 2]                 # USD camera looks along -Z
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, fwd[2]))))
    return abs(pitch) < 1.0 and abs(m[2][3] - 0.103) < 0.006


def score(pred, gt, valid):
    p = (pred > 0) & valid
    g = (gt > 0) & valid
    k = np.ones((3, 3), np.uint8)
    gd = cv2.dilate(g.astype(np.uint8), k) > 0
    pd = cv2.dilate(p.astype(np.uint8), k) > 0
    return int((p & gd).sum()), int(p.sum()), int((g & pd).sum()), int(g.sum())


def overlay(gt, valid, pred):
    h, w = gt.shape
    img = np.full((h, w, 3), 245, np.uint8)
    img[valid == 0] = (215, 215, 215)
    g = gt > 0
    p = pred > 0
    img[g] = (170, 170, 170)
    img[p & g] = (60, 160, 60)          # hit (green)
    img[p & ~cv2.dilate(g.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)] = (40, 40, 220)  # false (red)
    img[g & ~cv2.dilate(p.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)] = (220, 120, 30)  # missed (blue)
    return cv2.resize(img, (w * 2, h * 2), interpolation=cv2.INTER_NEAREST)   # BEV rows: far at top, cols: left..right


def render(M, f, path):
    """Camera frame + one labelled BEV overlay per method (green hit, red false, blue missed)."""
    from PIL import Image, ImageDraw, ImageFont
    z = np.load(f)
    bgr = z['rgb'][:, :, ::-1].copy(); gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gt, valid = z['bev_mask'], z['bev_valid'] > 0
    font_p = next((p for p in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
                               '/usr/share/fonts/truetype/nanum/NanumGothic.ttf'] if Path(p).exists()), None)
    fnt = ImageFont.truetype(font_p, 18) if font_p else ImageFont.load_default()
    tiles = []
    for n, plug in M:
        out = plug.process(bgr, gray, 0.0, vision.X_FAR)
        s = score(out.line, gt, valid)
        t = overlay(gt, z['bev_valid'], out.line)
        im = Image.new('RGB', (t.shape[1], t.shape[0] + 30), 'white')
        im.paste(Image.fromarray(t[:, :, ::-1]), (0, 30))
        ImageDraw.Draw(im).text((6, 4), f"{n}   P {s[0] / max(1, s[1]):.2f}  R {s[2] / max(1, s[3]):.2f}", fill='black', font=fnt)
        tiles.append(im)
    tw, th = tiles[0].size
    cam = Image.fromarray(z['rgb']).resize((int(th * 4 / 3), th))
    cols = 4
    W = max(cam.size[0] + tw * 3, tw * cols); rows = 1 + (len(tiles) - 3 + cols - 1) // cols
    canvas = Image.new('RGB', (W + 20, th * rows + 50), 'white')
    ImageDraw.Draw(canvas).text((8, 8), f"{f.parent.name}/{f.name}   정답 회색 · 맞음 초록 · 틀림 빨강 · 놓침 파랑 (위가 먼 곳)", fill='black', font=fnt)
    canvas.paste(cam, (8, 40))
    x = 8 + cam.size[0]
    for t in tiles[:3]:
        canvas.paste(t, (x, 40)); x += tw
    for i, t in enumerate(tiles[3:]):
        canvas.paste(t, (8 + (i % cols) * tw, 40 + th * (1 + i // cols)))
    canvas.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--maps', nargs='+', required=True)
    ap.add_argument('--frames', type=int, default=10)
    ap.add_argument('--images', type=int, default=4, help='comparison images of the frames where V0 is worst')
    ap.add_argument('--out', type=Path, required=True)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    M = [(n, make_perception(kind, FakeNode(p))) for n, kind, p in methods()]
    tot = {n: [0, 0, 0, 0] for n, _ in M}
    per_map = {n: {} for n, _ in M}
    used = {}
    worst = []          # (v0 false cells, map, file)
    for mp in a.maps:
        files = sorted((DATA / mp).glob('*.npz'))
        picked = []
        for f in files:
            z = np.load(f)
            if level(z['world_from_usd_camera']):
                picked.append(f)
        step = max(1, len(picked) // a.frames)
        picked = picked[::step][:a.frames]
        used[mp] = len(picked)
        for fi, f in enumerate(picked):
            z = np.load(f)
            bgr = z['rgb'][:, :, ::-1].copy()
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            gt, valid = z['bev_mask'], z['bev_valid'] > 0
            tiles = []
            for n, plug in M:
                out = plug.process(bgr, gray, 0.0, vision.X_FAR)
                s = score(out.line, gt, valid)
                if n == M[0][0]:
                    worst.append((s[1] - s[0], mp, str(f)))
                tot[n] = [x + y for x, y in zip(tot[n], s)]
                pm = per_map[n].setdefault(mp, [0, 0, 0, 0])
                per_map[n][mp] = [x + y for x, y in zip(pm, s)]

    worst.sort(reverse=True)
    for rank, (fp, mp, fpath) in enumerate(worst[:a.images]):
        render(M, Path(fpath), a.out / f'worst{rank + 1}_{mp}.png')
    res = {'maps': used, 'methods': [n for n, _ in M], 'pooled': {}, 'per_map_precision': {}}
    for n, _ in M:
        tp, pn, hit, gn = tot[n]
        P, R = tp / max(1, pn), hit / max(1, gn)
        res['pooled'][n] = {'precision': P, 'recall': R, 'f05': 1.25 * P * R / max(1e-9, 0.25 * P + R)}
        res['per_map_precision'][n] = {m: v[0] / max(1, v[1]) for m, v in per_map[n].items()}
    (a.out / 'results.json').write_text(json.dumps(res, ensure_ascii=False, indent=1))
    base = res['per_map_precision'][M[0][0]]
    lines = ['| 방법 | 정밀도 | 재현율 | F0.5 | V0 이상인 맵 | 최악 맵 정밀도 |', '|---|---|---|---|---|---|']
    for n, _ in M:
        r = res['pooled'][n]; pm = res['per_map_precision'][n]
        wins = sum(pm[m] >= base[m] - 1e-9 for m in pm) if n != M[0][0] else '–'
        lines.append(f"| {n} | {r['precision']:.3f} | {r['recall']:.3f} | {r['f05']:.3f} | {wins}{'' if wins == '–' else '/' + str(len(pm))} | {min(pm.values()):.3f} |")
    (a.out / 'results.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines)); print('frames per map', used)


if __name__ == '__main__':
    main()
