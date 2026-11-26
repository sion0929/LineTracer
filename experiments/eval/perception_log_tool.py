#!/usr/bin/env python3
"""Inspect and replay per-frame perception logs (for the planning team).

A perception log (written by line_follower/perception/log.py) is a directory with meta.json and
chunk_NNNN.npz. Each record is one camera frame that reached the map:
    t [s], pose (x, y, yaw) in odom at capture, pitch [rad], travel [m], used (map updated?),
    line / valid / far BEV masks of shape (BH, BW).
BEV grid: row 0 = x_far (ahead), last row = x_near, col 0 = +y_half (left); cell (r, c) centre is
    x = x_far - (r + 0.5) * res   (forward),   y = y_half - (c + 0.5) * res   (left).
World point = pose_xy + R(yaw) @ (x, y).

Usage (run with tools/pylf, numpy + opencv needed):
    tools/pylf experiments/eval/perception_log_tool.py summary LOG [LOG ...]
    tools/pylf experiments/eval/perception_log_tool.py rebuild LOG --out map.png [--npz map.npz] [--world]
    tools/pylf experiments/eval/perception_log_tool.py frames  LOG --out DIR [--every 10]

summary  markdown table: frames, used, duration, distance, mean line/valid cells per frame,
         fraction of frames without any line cell, pitch min/max [deg].
rebuild  re-project every *used* frame into a 5 mm odom grid, accumulating per-cell `seen`
         (valid or line) and `hits` (line). PNG: seen = light grey, hits/seen >= 0.5 = black, else
         dark grey, cropped to the bounding box + 0.1 m. --npz saves hits, seen, origin (x, y of
         cell [0, 0] corner, metres, row = x axis, col = y axis), res. --world converts odom to
         world with job.json `odom_origin` found in the run dir above LOG, and overlays the
         ground-truth centreline (red) when job.json's map JSON has `centerline`.
         Image orientation: world/odom +x to the right, +y up.
frames   every K-th frame as PNG (line white, valid grey, invalid black); the frame index, time and
         pose are in the file name.

As a library:  meta, d = load_log(dir);  d['line'] is (N, BH, BW) bool, d['pose'] (N, 3), ...
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
RES_OUT = 0.005
MARGIN = 0.1


def load_log(path):
    """Read a log directory -> (meta, dict of arrays; masks unpacked to bool (N, BH, BW))."""
    path = Path(path)
    meta = json.loads((path / 'meta.json').read_text())
    h, w = meta['grid']['shape']
    parts = [np.load(f) for f in sorted(path.glob('chunk_*.npz'))]
    if not parts:
        return meta, {}
    out = {k: np.concatenate([p[k] for p in parts]) for k in ('t', 'pose', 'pitch', 'travel', 'used')}
    out['used'] = out['used'].astype(bool)
    for k, fill in (('imu_pitch', np.nan), ('elev', np.nan), ('slope', -1)):   # format 2 only
        out[k] = np.concatenate([p[k] if k in p.files else np.full(len(p['t']), fill) for p in parts])
    for k in ('line', 'valid', 'far'):
        bits = np.concatenate([p[k] for p in parts])
        out[k] = np.unpackbits(bits, axis=1)[:, :h * w].reshape(-1, h, w).astype(bool)
    return meta, out


def bev_xy(meta):
    """Forward x and left y [m] of every BEV cell centre, each (BH, BW)."""
    g = meta['grid']
    h, w = g['shape']
    res = g['res_m']
    x = g['x_far_m'] - (np.arange(h) + 0.5) * res
    y = g['y_half_m'] - (np.arange(w) + 0.5) * res
    return np.broadcast_to(x[:, None], (h, w)), np.broadcast_to(y[None, :], (h, w))


# ---------------------------------------------------------------- summary
def summarize(path):
    meta, d = load_log(path)
    if not d:
        return {'log': str(path), 'frames': 0}
    t, pose, used = d['t'], d['pose'], d['used']
    xy = pose[:, :2]
    ok = np.isfinite(xy).all(1)
    dist = float(np.linalg.norm(np.diff(xy[ok], axis=0), axis=1).sum()) if ok.sum() > 1 else 0.0
    n_line = d['line'].sum((1, 2))
    pitch = np.degrees(d['pitch'][np.isfinite(d['pitch'])])
    return {'log': str(path), 'frames': len(t), 'used': int(used.sum()), 'duration_s': float(t[-1] - t[0]),
            'distance_m': dist, 'line_cells': float(n_line.mean()), 'valid_cells': float(d['valid'].sum((1, 2)).mean()),
            'no_line': float((n_line == 0).mean()),
            'pitch_min': float(pitch.min()) if pitch.size else float('nan'),
            'pitch_max': float(pitch.max()) if pitch.size else float('nan')}


def summary_table(paths):
    head = ('| log | frames | used | duration s | distance m | line cells/frame | valid cells/frame '
            '| frames w/o line | pitch min deg | pitch max deg |')
    rows = [head, '|' + '---|' * 10]
    for p in paths:
        s = summarize(p)
        name = Path(p).parent.name if Path(p).name == 'perception_log' else Path(p).name
        if not s['frames']:
            rows.append(f'| {name} | 0 | | | | | | | | |')
            continue
        rows.append(f"| {name} | {s['frames']} | {s['used']} | {s['duration_s']:.1f} | {s['distance_m']:.2f} "
                    f"| {s['line_cells']:.0f} | {s['valid_cells']:.0f} | {s['no_line']:.3f} "
                    f"| {s['pitch_min']:.2f} | {s['pitch_max']:.2f} |")
    return '\n'.join(rows)


# ---------------------------------------------------------------- rebuild
def find_job(path):
    """job.json of the run dir that holds the log (the log dir or one of its parents), else None."""
    p = Path(path).resolve()
    for q in [p, *p.parents[:3]]:
        if (q / 'job.json').exists():
            return json.loads((q / 'job.json').read_text())
    return None


def accumulate(meta, d, res=RES_OUT, odom_origin=None):
    """Splat used frames into a grid. Returns hits, seen (int32 (H, W); row = x, col = y), origin (x0, y0), res.
    odom_origin (x, y, yaw) converts the poses to world first."""
    bx, by = bev_xy(meta)
    use = np.flatnonzero(d['used'] & np.isfinite(d['pose']).all(1))
    if not len(use):
        return None
    pose = d['pose'][use].astype(float).copy()
    if odom_origin is not None:
        ox, oy, oyaw = odom_origin
        c, s = np.cos(oyaw), np.sin(oyaw)
        px, py = pose[:, 0].copy(), pose[:, 1].copy()
        pose[:, 0] = ox + c * px - s * py
        pose[:, 1] = oy + s * px + c * py
        pose[:, 2] += oyaw
    g = meta['grid']
    reach = float(np.hypot(max(abs(g['x_far_m']), abs(g['x_near_m'])), g['y_half_m'])) + res
    x0 = float(pose[:, 0].min() - reach)
    y0 = float(pose[:, 1].min() - reach)
    H = int(np.ceil((pose[:, 0].max() + reach - x0) / res)) + 1
    W = int(np.ceil((pose[:, 1].max() + reach - y0) / res)) + 1
    hits = np.zeros(H * W, np.int32)
    seen = np.zeros(H * W, np.int32)
    lm, vm = d['line'], d['valid']
    step = 100
    for a in range(0, len(use), step):
        idx_all, hit_all = [], []
        for k in use[a:a + step]:
            i = a + int(np.searchsorted(use[a:a + step], k))
            px, py, yaw = pose[i]
            m = vm[k] | lm[k]
            c, s = np.cos(yaw), np.sin(yaw)
            wx = px + c * bx[m] - s * by[m]
            wy = py + s * bx[m] + c * by[m]
            r = np.floor((wx - x0) / res).astype(np.int64)
            q = np.floor((wy - y0) / res).astype(np.int64)
            idx_all.append(r * W + q)
            hit_all.append(lm[k][m])
        idx = np.concatenate(idx_all)
        hit = np.concatenate(hit_all)
        seen += np.bincount(idx, minlength=H * W).astype(np.int32)
        hits += np.bincount(idx[hit], minlength=H * W).astype(np.int32)
    return hits.reshape(H, W), seen.reshape(H, W), (x0, y0), res


def render(hits, seen, origin, res, centerline=None):
    """BGR image, +x right, +y up, cropped to the seen bounding box + MARGIN."""
    import cv2
    rows, cols = np.nonzero(seen)
    m = int(round(MARGIN / res))
    r0, r1 = max(rows.min() - m, 0), min(rows.max() + m + 1, seen.shape[0])
    c0, c1 = max(cols.min() - m, 0), min(cols.max() + m + 1, seen.shape[1])
    s, h = seen[r0:r1, c0:c1], hits[r0:r1, c0:c1]
    g = np.full(s.shape, 255, np.uint8)
    g[s > 0] = 200
    g[(s > 0) & (h > 0)] = 90
    g[(s > 0) & (h >= 0.5 * np.maximum(s, 1))] = 0
    img = np.dstack([g, g, g])
    # array (row = x, col = y) -> image (x right, y up)
    img = np.ascontiguousarray(img.transpose(1, 0, 2)[::-1])
    if centerline is not None and len(centerline):
        cl = np.asarray(centerline, float)[:, :2]
        px = (cl[:, 0] - origin[0]) / res - r0
        py = (cl[:, 1] - origin[1]) / res - c0
        u = px
        v = (c1 - c0) - 1 - py
        pts = np.stack([u, v], 1).round().astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], False, (0, 0, 255), 1, cv2.LINE_AA)
    return img


def resolve_map(p):
    """Map path from job.json: '/ws/...' (docker mount) or repo-relative -> path under this repo."""
    p = str(p)
    if p.startswith('/ws/'):
        p = str(REPO / p[4:])
    elif not p.startswith('/'):
        p = str(REPO / p)
    return Path(p)


def cmd_rebuild(a):
    import cv2
    meta, d = load_log(a.log)
    if not d:
        sys.exit('empty log')
    origin_pose, cl = None, None
    if a.world:
        job = find_job(a.log)
        if not job or 'odom_origin' not in job:
            sys.exit('--world needs job.json with odom_origin in the run dir above the log')
        origin_pose = job['odom_origin']
        mp = resolve_map(job.get('map', ''))
        if mp.is_file():
            cl = json.loads(mp.read_text()).get('centerline')
    r = accumulate(meta, d, odom_origin=origin_pose)
    if r is None:
        sys.exit('no used frames with a pose')
    hits, seen, origin, res = r
    cv2.imwrite(a.out, render(hits, seen, origin, res, cl))
    if a.npz:
        np.savez_compressed(a.npz, hits=hits, seen=seen, origin=np.array(origin), res=res)
    print(f'wrote {a.out} ({"world" if a.world else "odom"} frame, {int(d["used"].sum())} used frames)')


# ---------------------------------------------------------------- frames
def cmd_frames(a):
    import cv2
    meta, d = load_log(a.log)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for i in range(0, len(d.get('t', [])), a.every):
        img = np.zeros(d['line'][i].shape, np.uint8)
        img[d['valid'][i]] = 110
        img[d['line'][i]] = 255
        x, y, yaw = d['pose'][i]
        cv2.imwrite(str(out / f'f{i:05d}_t{d["t"][i]:.2f}_x{x:.3f}_y{y:.3f}_yaw{yaw:.3f}.png'), img)
    print(f'wrote frames to {out}')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('summary')
    s.add_argument('logs', nargs='+')
    r = sub.add_parser('rebuild')
    r.add_argument('log')
    r.add_argument('--out', required=True)
    r.add_argument('--npz')
    r.add_argument('--world', action='store_true')
    f = sub.add_parser('frames')
    f.add_argument('log')
    f.add_argument('--out', required=True)
    f.add_argument('--every', type=int, default=10)
    a = ap.parse_args(argv)
    if a.cmd == 'summary':
        print(summary_table(a.logs))
    elif a.cmd == 'rebuild':
        cmd_rebuild(a)
    else:
        cmd_frames(a)


if __name__ == '__main__':
    main()
