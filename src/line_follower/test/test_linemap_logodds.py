import math

import numpy as np
import pytest

from line_follower import vision
from line_follower.line_map import C0, RES, LineMap


def observation(distance, hit, width=1):
    valid = np.zeros((vision.BH, vision.BW), np.uint8)
    row = round((vision.X_FAR - distance) / RES)
    col = round(vision.Y_HALF / RES)
    valid[row, col:col + width] = 255
    return (valid.copy() if hit else np.zeros_like(valid)), valid


def test_single_far_false_hit_is_cleared_by_near_floor():
    mapping = LineMap(map_mode='logodds')
    far_distance = round((vision.X_FAR - 0.04) / RES) * RES
    line, far = observation(far_distance, hit=True)
    mapping.update(line, far, (0.0, 0.0, 0.0), add_only=far, far_mask=far)
    weight = LineMap._bev_conf(far)[far > 0].item() / 255
    evidence = 0.85 * weight
    assert mapping.confidence(far_distance, 0.0) == pytest.approx(1 / (1 + math.exp(-evidence)))
    assert not mapping.hit(far_distance, 0.0)

    empty, near = observation(0.4, hit=False)
    for _ in range(2):
        mapping.update(empty, near, (far_distance - 0.4, 0.0, 0.0))
        evidence -= 0.4
        assert mapping.confidence(far_distance, 0.0) == pytest.approx(1 / (1 + math.exp(-evidence)))
    assert mapping.confidence(far_distance, 0.0) < 0.5
    assert not mapping.hit(far_distance, 0.0)


def test_consistent_far_line_survives_a_near_miss():
    mapping = LineMap(map_mode='logodds')
    distance = round((vision.X_FAR - 0.04) / RES) * RES
    line, far = observation(distance, hit=True)
    for _ in range(8):
        mapping.update(line, far, (0.0, 0.0, 0.0), add_only=far, far_mask=far)
    assert mapping.hit(distance, 0.0)

    near_line, near = observation(0.4, hit=True)
    pose = (distance - 0.4, 0.0, 0.0)
    for _ in range(8):
        mapping.update(near_line, near, pose)
    assert mapping.confidence(distance, 0.0) == pytest.approx(1 / (1 + math.exp(-3.5)))
    mapping.update(np.zeros_like(near_line), near, pose)
    assert mapping.hit(distance, 0.0)


def test_add_only_floor_leaves_existing_evidence_unchanged():
    mapping = LineMap(map_mode='logodds')
    line, valid = observation(0.4, hit=True)
    pose = (0.0, 0.0, 0.0)
    mapping.update(line, valid, pose)
    before = mapping.confidence(0.4, 0.0)
    mapping.update(np.zeros_like(line), valid, pose, add_only=valid, far_mask=valid)
    assert mapping.confidence(0.4, 0.0) == before
    assert mapping.hit(0.4, 0.0)


@pytest.mark.parametrize('initial_hit', [False, True])
def test_frozen_cells_keep_evidence(initial_hit):
    mapping = LineMap(map_mode='logodds')
    pose = (0.0, 0.0, 0.0)
    line, valid = observation(0.4, hit=initial_hit)
    mapping.update(line, valid, pose)
    before = mapping.confidence(0.4, 0.0)
    mapping.freeze(0.4, 0.0, 0.02)
    other, _ = observation(0.4, hit=not initial_hit)
    for _ in range(4):
        mapping.update(other, valid, pose)
    assert mapping.confidence(0.4, 0.0) == before
    assert mapping.hit(0.4, 0.0) == initial_hit


def test_default_and_explicit_legacy_match():
    default = LineMap()
    explicit = LineMap(map_mode='legacy')
    line, valid = observation(0.4, hit=True, width=3)
    for mapping in (default, explicit):
        mapping.update(line, valid, (0.0, 0.0, 0.0), add_only=valid, far_mask=valid)
        assert mapping.hit(0.4, 0.0)
        assert mapping.seen[C0, C0 + round(0.4 / RES)] == 255
    for layer in ('grid', 'seen', 'conf'):
        np.testing.assert_array_equal(getattr(default, layer), getattr(explicit, layer))
