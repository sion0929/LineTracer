"""Synthetic smoke test for experiments/jetson/bench_stages.py (no ROS, no real data)."""
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/line_follower'))
sys.path.insert(0, str(ROOT / 'experiments/jetson'))

from bench_stages import (bench_map_update, bench_perception,  # noqa: E402
                          bench_perception_total, bench_planning, load_frames)
from line_follower import vision  # noqa: E402
from line_follower.perception.log import PerceptionLog  # noqa: E402


class _P:
    def __init__(self, value):
        self.value = value


class _Node:
    def __init__(self, params):
        self.p = params

    def has_parameter(self, name):
        return name in self.p

    def get_parameter(self, name):
        return _P(self.p[name])


def _write_frames(directory, count=30):
    directory.mkdir()
    for i in range(count):
        gray = np.full((vision.H, vision.W), 200, np.uint8)
        centre = int(round(vision.W * 0.5 + 20 * np.sin(0.2 * i)))
        cv2.rectangle(gray, (centre - 8, 120), (centre + 8, vision.H - 1), 30, -1)
        rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
        np.savez_compressed(directory / f'{i:06d}.npz', rgb=rgb)


def _write_log(directory, count=40):
    log = PerceptionLog(_Node({'perception': 'rule'}), str(directory))
    for i in range(count):
        line = np.zeros((vision.BH, vision.BW), np.uint8)
        for row in range(vision.BH):
            col = int(round(vision.BW * 0.5 + 10 * np.sin(0.05 * row + 0.05 * i)))
            line[row, max(0, col - 2):min(vision.BW, col + 3)] = 255
        valid = np.full_like(line, 255)
        travel = 0.0067 * i
        log.record(i / 30.0, (travel, 0.0, 0.0), 0.0, travel, True, line, valid)
    log.flush()


def _check(samples):
    assert len(samples) > 0
    values = np.asarray(samples, dtype=float)
    assert np.isfinite(values).all()
    assert (values >= 0.0).all()


def test_bench_stages_synthetic(tmp_path):
    frames_dir = tmp_path / 'frames'
    log_dir = tmp_path / 'log'
    _write_frames(frames_dir)
    _write_log(log_dir)

    frames = load_frames(frames_dir)
    assert len(frames) == 30

    _check(bench_perception(frames, max_frames=30))
    _check(bench_perception_total(frames, max_frames=30))
    _check(bench_map_update(log_dir, max_frames=40))
    _check(bench_planning(log_dir, max_frames=40))


@pytest.mark.parametrize('stage', ['perception', 'perception_total', 'map_update', 'planning'])
def test_each_stage_reports_finite_timings(tmp_path, stage):
    frames_dir = tmp_path / 'frames'
    log_dir = tmp_path / 'log'
    _write_frames(frames_dir, 6)
    _write_log(log_dir, 8)
    frames = load_frames(frames_dir)
    if stage == 'perception':
        samples = bench_perception(frames, warmup=2)
    elif stage == 'perception_total':
        samples = bench_perception_total(frames, warmup=2)
    elif stage == 'map_update':
        samples = bench_map_update(log_dir, warmup=2)
    else:
        samples = bench_planning(log_dir, warmup=2)
    _check(samples)
