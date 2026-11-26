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

    assert np.allclose(z[xs < 0.44], 0.0)
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
