"""SlopeTracker: hill state sequence and the deck ground model (no ROS)."""
import copy
import math

import numpy as np
import pytest

from line_follower.perception.slope import SlopeTracker

DEG = math.pi / 180.0
STEP = 0.005
RAMP = 0.44
X0 = 0.5
PITCH = 7.8 * DEG


def _pitch(x):
    if x < X0:
        return 0.0
    if x < X0 + RAMP:
        return PITCH
    if x < X0 + 2 * RAMP:
        return 0.0
    if x < X0 + 3 * RAMP:
        return -PITCH
    return 0.0


def _run():
    tracker = SlopeTracker()
    xs = np.arange(0.0, X0 + 4 * RAMP, STEP)
    elev = 0.0
    states = []
    deck_start = None
    deck_elev = None
    prev_state = tracker.state
    for k, x in enumerate(xs):
        pitch = _pitch(x)
        ds = 0.0 if k == 0 else STEP
        elev = max(0.0, elev + ds * math.sin(pitch))
        state = tracker.update(ds, pitch, elev)
        if state == 'deck' and prev_state != 'deck':
            deck_start = copy.deepcopy(tracker)
            deck_elev = elev
        states.append(state)
        prev_state = state
    return tracker, states, deck_start, deck_elev


def test_state_sequence_flat_climb_deck_descent_flat():
    _, states, _, _ = _run()
    runs = [states[0]]
    for s in states[1:]:
        if s != runs[-1]:
            runs.append(s)
    assert runs == ['flat', 'climb', 'deck', 'descent', 'flat']


def test_climb_length_and_slope():
    tracker, _, _, _ = _run()
    assert tracker.climb_len == pytest.approx(RAMP, abs=0.02)
    assert abs(tracker.climb_slope - PITCH) < 0.5 * DEG


def test_deck_ground_models_the_far_edge():
    tracker, _, deck_start, deck_elev = _run()
    assert deck_start is not None
    assert tracker.deck_travel == pytest.approx(RAMP, abs=0.02)

    xs = np.array([0.0, 0.2, 0.43, 0.44, 0.5, 0.6, 0.8, 0.9, 1.2])
    z = deck_start.deck_ground(xs, deck_elev)

    assert np.allclose(z[xs < 0.42], 0.0, atol=1e-3)     # edge estimated within one 5 mm step
    ramp = (xs > 0.44) & (xs < 0.44 + 0.44)
    assert ramp.sum() >= 3
    assert np.all(z[ramp] < 0.0)
    assert np.all(np.diff(z[ramp]) < 0.0)
    assert np.allclose(z[xs > 0.44 + 0.44], -deck_elev, atol=0.002)


def test_deck_travel_advances_and_edge_offset_moves_the_edge():
    tracker, _, deck_start, deck_elev = _run()
    a = deck_start.deck_ground(np.array([0.55]), deck_elev)[0]
    b = deck_start.deck_ground(np.array([0.55]), deck_elev, edge_offset=0.05)[0]
    assert b > a                                  # a farther edge keeps the ground higher

    t = SlopeTracker()
    t.state = 'deck'
    t.deck_travel = 0.0
    t.climb_len = RAMP
    t.climb_slope = PITCH
    t.update(0.01, 0.0, deck_elev)
    assert t.deck_travel == pytest.approx(0.01)
    t.update(0.01, 0.0, deck_elev)
    assert t.deck_travel == pytest.approx(0.02)


def test_deck_edge_with_pitch_lag():
    """Pitch eases in and out over 0.12 m (wheelbase), as in Isaac and on the robot: the deck is
    noticed late, but the edge estimate still lands within 4 cm of the true edge (Isaac logs: 0.5 cm)."""
    ease = 0.12

    def pitch(x):
        def ramp_up(u):          # 0 -> PITCH over `ease`, centred on the breakpoint
            return PITCH * min(1.0, max(0.0, (u + ease / 2) / ease))
        a, b, c, d = X0, X0 + RAMP, X0 + 2 * RAMP, X0 + 3 * RAMP
        return ramp_up(x - a) - ramp_up(x - b) - ramp_up(x - c) + ramp_up(x - d)

    t = SlopeTracker()
    elev, x = 0.0, 0.0
    while x < X0 + 4 * RAMP:
        p = pitch(x)
        elev = max(0.0, elev + STEP * math.sin(p))
        if t.update(STEP, p, elev) == 'deck' and t.deck_travel == 0.0:
            true_edge = X0 + 2 * RAMP - x
            z = t.deck_ground(np.array([true_edge - 0.05, true_edge + 0.2]), elev)
            assert abs(math.degrees(t.climb_slope) - 7.8) < 0.3
            assert t.up_len == pytest.approx(RAMP, abs=0.02)
            assert z[0] == pytest.approx(0.0, abs=2e-3)                       # still deck
            assert z[1] == pytest.approx(-0.2 * math.tan(PITCH), abs=6e-3)    # on the down-ramp (4 cm)
            return
        x += STEP
    pytest.fail('deck never detected')


def test_middle_band_keeps_centre_and_clears_ramp_edges():
    from line_follower import vision
    from line_follower.perception.slope import middle_band
    line = np.zeros((vision.BH, vision.BW), np.uint8)
    col = lambda y: int(round((vision.Y_HALF - y) / vision.RES - 0.5))
    for y in (0.0, 0.33, -0.33, 0.15):
        line[:, col(y)] = 255
    out = middle_band(line, 0.22)
    assert out[:, col(0.0)].all() and out[:, col(0.15)].all()
    assert not out[:, col(0.33)].any() and not out[:, col(-0.33)].any()


def test_hill_memory_masks_the_climbed_ramp_only():
    from line_follower.perception.slope import HillMemory
    mem = HillMemory()
    mem.update('flat', (0.0, 0.0, 0.0))
    mem.update('climb', (1.0, 2.0, 0.0))           # ramp starts at (1, 2) heading +x
    mem.update('deck', (1.5, 2.0, 0.0), up_len=0.44)
    assert mem.hills[0][3] == pytest.approx(3 * 0.44 + 0.08)
    bx = np.array([[0.5, 0.5, 0.5, 2.0]])
    by = np.array([[0.0, 0.30, 0.50, 0.0]])
    m = mem.mask((1.2, 2.0, 0.0), bx, by)         # robot back on flat ground looking along the ramp
    assert m[0, 0] and m[0, 1] and not m[0, 2] and not m[0, 3]
