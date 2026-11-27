"""Perception output log (dev/analysis): what perception hands to planning, frame by frame.

Enabled by the node parameter `perception_log` (a directory) or, when that is empty, the
environment variable LF_PERCEPTION_LOG. Every camera frame that reaches the map writes one record:

    t (s), pose at capture (x, y, yaw in odom), pitch (rad), travel (m), used (map updated?),
    line / valid / far BEV masks (vision grid, packed bits),
    imu_pitch (filtered body pitch, rad), elev (wheel height above the start floor, m),
    slope (hill state: 0 flat, 1 climb, 2 deck, 3 descent, -1 unknown)   [format 2]

Records go to chunk_NNNN.npz every `CHUNK` frames (so an aborted run loses at most ~2 s) and
meta.json describes the grid and the perception settings. Planning replays these logs to design
its own filtering of noisy observations (experiments/eval/perception_log_tool.py).
Never raises: a logging failure disables the log and leaves driving untouched.
"""
import json
import os
import time
from pathlib import Path

import numpy as np

from .. import vision

CHUNK = 60
SLOPE_CODES = {'flat': 0, 'climb': 1, 'deck': 2, 'descent': 3}
OPTIONAL = {'imu_pitch': np.nan, 'elev': np.nan, 'slope': -1}   # absent in format-1 logs


class PerceptionLog:
    def __init__(self, node, path=''):
        self.dir = Path(path or os.environ.get('LF_PERCEPTION_LOG', '')) if (path or os.environ.get('LF_PERCEPTION_LOG')) else None
        self.rows, self.n_chunk, self.frames = [], 0, 0
        self.ok = self.dir is not None
        if not self.ok:
            return
        try:
            self.dir.mkdir(parents=True, exist_ok=True)
            gp = node.get_parameter

            def val(name, default=None):
                try:
                    return gp(name).value if node.has_parameter(name) else default
                except Exception:
                    return default
            meta = {
                'format': 'lf-perception-log/2',
                'created_unix': time.time(),
                'grid': {'res_m': vision.RES, 'x_near_m': vision.X_NEAR, 'x_far_m': vision.X_FAR,
                         'y_half_m': vision.Y_HALF, 'shape': [vision.BH, vision.BW],
                         'rows': 'row 0 = x_far (ahead), last row = x_near', 'cols': 'col 0 = +y_half (left)'},
                'settings': {k: val(k) for k in ('perception', 'near_scale', 'far_band', 'far_split', 'far_model',
                                                 'map_mode', 'freeze_passed', 'restore_mode', 'rule.max_width_m',
                                                 'depth.model_path', 'depth.splat', 'imu_floor')},
                'fields': {'t': 'capture time [s]', 'pose': 'x, y, yaw at capture (odom) [m, rad]',
                           'pitch': 'camera pitch used [rad]', 'travel': 'odometry distance [m]',
                           'used': 'frame written to the map', 'line/valid/far': 'np.packbits of BEV masks',
                           'imu_pitch': 'filtered IMU body pitch, nose up > 0 [rad]',
                           'elev': 'wheel height above the start floor from odometry + IMU [m]',
                           'slope': 'hill state ' + str(SLOPE_CODES) + ', -1 unknown'},
            }
            (self.dir / 'meta.json').write_text(json.dumps(meta, indent=1, default=str))
        except Exception:
            self.ok = False

    def record(self, t, pose, pitch, travel, used, line, valid, far=None, imu_pitch=np.nan, elev=np.nan, slope=None):
        if not self.ok:
            return
        try:
            x, y, yaw = (pose[0], pose[1], pose[2]) if pose is not None else (np.nan, np.nan, np.nan)
            self.rows.append((float(t), float(x), float(y), float(yaw), float(pitch if pitch is not None else np.nan),
                              float(travel), bool(used), np.packbits(line > 0), np.packbits(valid > 0),
                              np.packbits(far > 0) if far is not None else np.packbits(np.zeros_like(valid, bool)),
                              float(imu_pitch), float(elev), SLOPE_CODES.get(slope, -1)))
            self.frames += 1
            if len(self.rows) >= CHUNK:
                self.flush()
        except Exception:
            self.ok = False

    def flush(self):
        if not self.ok or not self.rows:
            return
        try:
            r = self.rows
            np.savez_compressed(
                self.dir / f'chunk_{self.n_chunk:04d}.npz',
                t=np.array([a[0] for a in r]), pose=np.array([a[1:4] for a in r]), pitch=np.array([a[4] for a in r]),
                travel=np.array([a[5] for a in r]), used=np.array([a[6] for a in r]),
                line=np.stack([a[7] for a in r]), valid=np.stack([a[8] for a in r]), far=np.stack([a[9] for a in r]),
                imu_pitch=np.array([a[10] for a in r]), elev=np.array([a[11] for a in r]),
                slope=np.array([a[12] for a in r], np.int8))
            self.n_chunk += 1
            self.rows = []
        except Exception:
            self.ok = False


def load(path):
    """Read a log directory -> (meta, dict of arrays with unpacked bool masks, shape (N, BH, BW))."""
    path = Path(path)
    meta = json.loads((path / 'meta.json').read_text())
    h, w = meta['grid']['shape']
    parts = [np.load(f) for f in sorted(path.glob('chunk_*.npz'))]
    if not parts:
        return meta, {}
    out = {k: np.concatenate([p[k] for p in parts]) for k in ('t', 'pose', 'pitch', 'travel', 'used')}
    for k, fill in OPTIONAL.items():   # format-1 chunks lack these: fill with the 'unknown' value
        out[k] = np.concatenate([p[k] if k in p.files else np.full(len(p['t']), fill) for p in parts])
    for k in ('line', 'valid', 'far'):
        bits = np.concatenate([p[k] for p in parts])
        out[k] = np.unpackbits(bits, axis=1)[:, :h * w].reshape(-1, h, w).astype(bool)
    return meta, out
