#!/usr/bin/env python3
"""Per-stage wall-clock benchmark for the line follower (no ROS).

Times the three CPU stages on the same machine so a PC run and a Jetson run can be
compared directly:

    perception      rule plugin on Isaac camera frames (BEV projection)
    perception_total  RGB->BGR/gray conversion + the rule plugin
    map_update      LineMap.update on a replayed perception log
    planning        RulePlanner.replan on the map as it grows from the same log

Only numpy, OpenCV and src/line_follower/line_follower are imported (no rclpy, no
torch, no scipy), so the script runs unchanged on the Jetson.

    tools/pylf experiments/jetson/bench_stages.py \
        --frames DIR --log DIR [--max-frames 300] --out out.json
"""
import argparse
import json
import os
import platform
import sys
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/line_follower'))

from line_follower import node_core, vision  # noqa: E402
from line_follower.line_map import LineMap  # noqa: E402
from line_follower.perception import make_perception  # noqa: E402
from line_follower.perception.log import load as load_log  # noqa: E402
from line_follower.planning import make_planner  # noqa: E402

WARMUP = 20
PERCEPTION = 'rule'


class _P:
    def __init__(self, value):
        self.value = value


class FakeNode:
    """Just enough of rclpy.Node for the perception plugins."""

    def __init__(self, params=None):
        self.p = dict(params or {})

    def declare_parameter(self, name, default=None, *a, **k):
        self.p.setdefault(name, default)
        return _P(self.p[name])

    def get_parameter(self, name):
        return _P(self.p[name])

    def has_parameter(self, name):
        return name in self.p

    def get_logger(self):
        class _Logger:
            def info(self, *a, **k):
                pass

            warning = error = warn = debug = info

        return _Logger()


class FakeController:
    def confirm_finish(self, path, observation):
        return None


def _declared_defaults():
    """All follower_node parameter defaults, without ROS.

    node_core imports cleanly without rclpy (its msg imports are optional), but
    ParameterDescriptor is then None and rng() would fail. Replace the two message
    classes with no-op stubs just while declare_follower_parameters runs.
    """
    descriptor, float_range = node_core.ParameterDescriptor, node_core.FloatingPointRange

    class _Stub:
        def __init__(self, *a, **k):
            pass

    node_core.ParameterDescriptor = node_core.FloatingPointRange = _Stub
    try:
        node = FakeNode()
        node_core.declare_follower_parameters(node)
        return dict(node.p)
    finally:
        node_core.ParameterDescriptor, node_core.FloatingPointRange = descriptor, float_range


class FakePlanningNode(FakeNode):
    """The state src/line_follower/line_follower/planning/rule.py actually reads."""

    def __init__(self):
        super().__init__(_declared_defaults())
        self.map = LineMap()
        self.pose = (0.0, 0.0, 0.0)
        self.travel = 0.0
        self.path = []
        self.path_end = 'open'
        self.t_point = None
        self.restorer = None
        self.controller = FakeController()
        self.last_observation = None
        self.n_restored = 0

    def to_odom(self, x, y, pose=None):
        px, py, th = pose or self.pose
        c, s = np.cos(th), np.sin(th)
        return (px + x * c - y * s, py + x * s + y * c)

    def to_robot(self, X, Y, pose=None):
        px, py, th = pose or self.pose
        dx, dy = X - px, Y - py
        c, s = np.cos(th), np.sin(th)
        return (dx * c + dy * s, -dx * s + dy * c)


def load_frames(frames_dir, max_frames=None):
    """Isaac *.npz frames -> list of HxWx3 uint8 RGB arrays (key 'rgb')."""
    files = sorted(Path(frames_dir).glob('*.npz'))
    if not files:
        raise FileNotFoundError(f'no *.npz frames in {frames_dir}')
    if max_frames is not None:
        files = files[:max_frames]
    frames = []
    for path in files:
        with np.load(path) as z:
            if 'rgb' not in z.files:
                raise KeyError(f'{path} has no "rgb" key')
            rgb = z['rgb']
            if rgb.ndim != 3 or rgb.shape[2] != 3:
                raise ValueError(f'{path}: rgb shape {rgb.shape} is not HxWx3')
            frames.append(np.ascontiguousarray(rgb))
    return frames


def _inputs(rgb):
    bgr = rgb[:, :, ::-1].copy()
    return bgr, cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)


def _limit(items, max_frames):
    return items[:max_frames] if max_frames is not None else items


def bench_perception(frames, max_frames=None, warmup=WARMUP):
    """Milliseconds of plug.process() per frame (conversions done before the clock)."""
    frames = _limit(frames, max_frames)
    if not frames:
        return []
    plug = make_perception(PERCEPTION, FakeNode({}))
    for k in range(warmup):
        bgr, gray = _inputs(frames[k % len(frames)])
        plug.process(bgr, gray, 0.0, vision.X_FAR)
    samples = []
    for rgb in frames:
        bgr, gray = _inputs(rgb)
        t0 = perf_counter()
        plug.process(bgr, gray, 0.0, vision.X_FAR)
        samples.append((perf_counter() - t0) * 1e3)
    return samples


def bench_perception_total(frames, max_frames=None, warmup=WARMUP):
    """Milliseconds of RGB->BGR, BGR->gray and plug.process() together."""
    frames = _limit(frames, max_frames)
    if not frames:
        return []
    plug = make_perception(PERCEPTION, FakeNode({}))
    for k in range(warmup):
        bgr, gray = _inputs(frames[k % len(frames)])
        plug.process(bgr, gray, 0.0, vision.X_FAR)
    samples = []
    for rgb in frames:
        t0 = perf_counter()
        bgr, gray = _inputs(rgb)
        plug.process(bgr, gray, 0.0, vision.X_FAR)
        samples.append((perf_counter() - t0) * 1e3)
    return samples


def _log_rows(log_dir, max_frames=None):
    """Used log frames -> [(line_u8, valid_u8, pose, travel), ...]."""
    _meta, d = load_log(log_dir)
    if not d:
        return []
    idx = np.nonzero(d['used'])[0]
    idx = _limit(idx, max_frames)
    rows = []
    for i in idx:
        line = np.ascontiguousarray(d['line'][i].astype(np.uint8) * 255)
        valid = np.ascontiguousarray(d['valid'][i].astype(np.uint8) * 255)
        pose = tuple(float(v) for v in d['pose'][i])
        rows.append((line, valid, pose, float(d['travel'][i])))
    return rows


def bench_map_update(log_dir, max_frames=None, warmup=WARMUP):
    """Milliseconds of LineMap.update() per used log frame on a fresh map."""
    rows = _log_rows(log_dir, max_frames)
    if not rows:
        return []
    warm_map = LineMap()
    for k in range(warmup):
        line, valid, pose, _travel = rows[k % len(rows)]
        warm_map.update(line, valid, pose)
    mapping = LineMap()
    samples = []
    for line, valid, pose, _travel in rows:
        t0 = perf_counter()
        mapping.update(line, valid, pose)
        samples.append((perf_counter() - t0) * 1e3)
    return samples


def bench_planning(log_dir, max_frames=None, warmup=WARMUP):
    """Milliseconds of RulePlanner.replan(), replaying map updates between calls."""
    rows = _log_rows(log_dir, max_frames)
    if not rows:
        return []

    def replay(count, timed):
        node = FakePlanningNode()
        planner = make_planner('rule', node)
        samples = []
        for k in range(count):
            line, valid, pose, travel = rows[k % len(rows)]
            node.map.update(line, valid, pose)
            node.pose, node.travel = pose, travel
            if timed:
                t0 = perf_counter()
                planner.replan()
                samples.append((perf_counter() - t0) * 1e3)
            else:
                planner.replan()
        return samples

    replay(warmup, timed=False)
    return replay(len(rows), timed=True)


def summarize(samples):
    if not samples:
        return {'n': 0, 'mean_ms': None, 'p50_ms': None, 'p95_ms': None, 'max_ms': None}
    a = np.asarray(samples, dtype=float)
    p50, p95 = np.percentile(a, [50, 95])
    return {'n': int(a.size), 'mean_ms': float(a.mean()), 'p50_ms': float(p50),
            'p95_ms': float(p95), 'max_ms': float(a.max())}


def machine_info():
    return {
        'platform': platform.platform(),
        'machine': platform.machine(),
        'processor': platform.processor(),
        'cpu_count': os.cpu_count(),
        'python': platform.python_version(),
        'numpy': np.__version__,
        'opencv': cv2.__version__,
        'opencv_threads': cv2.getNumThreads(),
    }


def _table(stages):
    lines = ['| stage | n | mean ms | p50 ms | p95 ms | max ms |',
             '|---|---:|---:|---:|---:|---:|']
    for name, s in stages.items():
        if s['n']:
            lines.append(f"| {name} | {s['n']} | {s['mean_ms']:.3f} | {s['p50_ms']:.3f} | "
                         f"{s['p95_ms']:.3f} | {s['max_ms']:.3f} |")
        else:
            lines.append(f'| {name} | 0 | - | - | - | - |')
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--frames', type=Path, required=True, help='Isaac *.npz frame dir (key rgb)')
    ap.add_argument('--log', type=Path, required=True, help='perception log dir')
    ap.add_argument('--max-frames', type=int, default=None,
                    help='frames per stage (default: all available)')
    ap.add_argument('--out', type=Path, required=True, help='output JSON')
    args = ap.parse_args(argv)

    frames = load_frames(args.frames, args.max_frames)
    stages = {
        'perception': summarize(bench_perception(frames, args.max_frames)),
        'perception_total': summarize(bench_perception_total(frames, args.max_frames)),
        'map_update': summarize(bench_map_update(args.log, args.max_frames)),
        'planning': summarize(bench_planning(args.log, args.max_frames)),
    }
    info = machine_info()
    result = {
        'machine': info,
        'inputs': {'frames': str(args.frames), 'log': str(args.log),
                   'max_frames': args.max_frames, 'frames_loaded': len(frames)},
        'stages': stages,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=1))

    print(f"machine={info['machine']} cpus={info['cpu_count']} python={info['python']} "
          f"numpy={info['numpy']} opencv={info['opencv']}")
    print(f"frames={len(frames)} log={args.log} max_frames={args.max_frames}")
    print('\n'.join(_table(stages)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
