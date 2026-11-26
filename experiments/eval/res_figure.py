#!/usr/bin/env python3
"""Side-by-side grid like the Isaac preview: 640 RGB | 640 mask | 320 RGB | 320 mask | 640 BEV | 320 BEV."""
import os, sys
from pathlib import Path
os.environ["LF_X_FAR"] = "1.6"
import cv2, numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/line_follower"))
from line_follower import vision  # noqa: E402

D = ROOT / "experiments/results/lead/res_compare"
picks = sys.argv[1:] or ["tile_034_ex0", "tile_034_ex2", "tile_009_ex1", "tile_009_ex3"]
TW, TH = 320, 240


def mask_of(g):
    thr = min(125.0, 0.62 * float(np.median(g[int(g.shape[0] * 0.55):])))
    return ((g < thr) * 255).astype(np.uint8)


def label(img, text):
    img = img.copy()
    cv2.putText(img, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 230, 255), 1, cv2.LINE_AA)
    return img


def bev_tile(m640):
    bev, valid = vision.bev_warp(m640)
    out = cv2.cvtColor(np.where(valid > 0, bev, 0).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    out[valid == 0] = (40, 40, 40)
    # distance ticks every 0.25 m
    for d in np.arange(0.5, 1.61, 0.25):
        r = int((vision.X_FAR - d) / vision.RES)
        cv2.line(out, (0, r), (12, r), (0, 160, 255), 2)
        cv2.putText(out, f"{d:.2f}m", (14, r + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 160, 255), 1, cv2.LINE_AA)
    return cv2.resize(out, (int(TH * out.shape[1] / out.shape[0]), TH), interpolation=cv2.INTER_AREA)


rows = []
for name in picks:
    bgr = cv2.imread(str(D / f"{name}.png"))
    g6 = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    small = cv2.resize(bgr, (320, 240), interpolation=cv2.INTER_AREA)
    g3 = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    m6, m3 = mask_of(g6), mask_of(g3)
    m3up = cv2.resize(m3, (640, 480), interpolation=cv2.INTER_NEAREST)
    # crop the far band (around the horizon) is where the difference is: show full frame, but tiles are 320x240
    tiles = [
        label(cv2.resize(bgr, (TW, TH), interpolation=cv2.INTER_AREA), f"{name} / 640x480 RGB"),
        label(cv2.cvtColor(cv2.resize(m6, (TW, TH), interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR), "640 line mask"),
        label(cv2.resize(small, (TW, TH), interpolation=cv2.INTER_NEAREST), "320x240 RGB"),
        label(cv2.cvtColor(m3, cv2.COLOR_GRAY2BGR), "320 line mask"),
        label(bev_tile(m6), "640 BEV to 1.6 m"),
        label(bev_tile(m3up), "320 BEV to 1.6 m"),
    ]
    rows.append(np.hstack(tiles))
    # zoom of the far band (rows 230-300 of 640 image) for both
    band6 = g6[236:300, 120:520]
    band3 = cv2.resize(g3[118:150, 60:260], (400, 64), interpolation=cv2.INTER_NEAREST)
    mb6 = m6[236:300, 120:520]
    mb3 = cv2.resize(m3[118:150, 60:260], (400, 64), interpolation=cv2.INTER_NEAREST)
    z = np.hstack([np.vstack([band6, mb6]), np.full((128, 8), 255, np.uint8), np.vstack([band3, mb3])])
    z = cv2.cvtColor(z, cv2.COLOR_GRAY2BGR)
    z = cv2.resize(z, (rows[-1].shape[1], int(128 * rows[-1].shape[1] / z.shape[1])), interpolation=cv2.INTER_NEAREST)
    cv2.putText(z, "far band zoom: 640 (left) vs 320 (right), image + mask", (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 120, 255), 2, cv2.LINE_AA)
    rows.append(z)
w = max(r.shape[1] for r in rows)
grid = np.vstack([np.pad(r, ((0, 4), (0, w - r.shape[1]), (0, 0)), constant_values=255) for r in rows])
cv2.imwrite(str(D / "res_grid.png"), grid)
print(grid.shape)
