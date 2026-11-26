import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hill_zone_eval as hze  # noqa: E402


def test_heading_ramp_zones():
    ramps = [dict(x0=2.75, y0=1.10, heading=math.pi / 2,
                  up=0.44, deck=0.44, down=0.44, height_m=0.06)]
    points = np.array([
        [2.75, 1.30],  # up
        [2.75, 1.70],  # deck
        [3.05, 1.70],  # deck, inside the ramp width
        [2.75, 2.10],  # down
        [2.75, 0.90],  # approach before
        [2.75, 2.60],  # approach after
        [2.75, 3.00],  # flat (beyond the approach)
        [3.20, 1.70],  # flat (outside the width + margin)
    ])
    zones = hze.classify_zones(ramps, points)
    assert list(zones) == ["up", "deck", "deck", "down",
                           "approach", "approach", "flat", "flat"]


def test_legacy_official_ramp_zones():
    ramps = hze.official_ramps()
    points = np.array([
        [0.80, 0.30],  # up
        [1.30, 0.30],  # deck
        [1.70, 0.30],  # down
        [0.50, 0.30],  # approach before
        [2.10, 0.30],  # approach after
        [0.10, 0.30],  # flat (before the approach)
        [1.30, 0.80],  # flat (outside the y width + margin)
    ])
    zones = hze.classify_zones(ramps, points)
    assert list(zones) == ["up", "deck", "down", "approach",
                           "approach", "flat", "flat"]


def test_synthetic_deck_precision():
    meta = {"centerline": [[-0.5, 0.0], [1.1, 0.0]], "line_width_m": 0.02}
    ramps = [dict(x0=0.0, y0=0.0, heading=0.0,
                  up=0.2, deck=0.2, down=0.2, height_m=0.06)]
    on_line = np.array([
        [-0.45, 0.0],  # flat before
        [-0.20, 0.0],  # approach before
        [0.10, 0.0],   # up
        [0.30, 0.0],   # deck
        [0.50, 0.0],   # down
        [0.80, 0.0],   # approach after
        [1.05, 0.0],   # flat after
    ])
    wrong = np.array([[0.25, 0.20], [0.32, 0.20], [0.38, 0.20]])
    metrics = hze.score_map_cells(np.vstack([on_line, wrong]), meta, ramps)

    assert metrics["deck"]["cells"] == 4
    assert metrics["deck"]["precision"] < 1.0
    for zone in ("up", "down", "approach", "flat"):
        assert metrics[zone]["precision"] == 1.0
    assert metrics["all"]["precision"] < 1.0
