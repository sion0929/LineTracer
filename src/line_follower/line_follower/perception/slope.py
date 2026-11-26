"""Hill state from the IMU pitch and a ground model for the raised deck.

Pure Python (numpy only, no ROS): the perception stage feeds the filtered pitch and the
travelled distance, and asks for the ground height ahead while the robot stands on the deck.
"""
import math

import numpy as np


class SlopeTracker:
    """Hill state from the IMU pitch: 'flat' -> 'climb' -> 'deck' -> 'descent' -> 'flat'."""

    def __init__(self, on_deg=3.0, off_deg=1.5, min_elev=0.01):
        self.on = math.radians(on_deg)
        self.off = math.radians(off_deg)
        self.min_elev = float(min_elev)
        self.state = 'flat'
        self.climb_len = 0.0          # distance driven while climbing [m]
        self.climb_slope = 0.0        # mean pitch while climbing [rad]
        self.deck_travel = 0.0        # distance driven since entering the deck [m]
        self._climb_pitch_sum = 0.0   # for the distance-weighted mean pitch

    def update(self, ds, pitch, elev):
        """Advance by ds [m] with the current pitch [rad] and elevation [m]; returns the state."""
        ds = max(0.0, float(ds))
        pitch = float(pitch)
        elev = float(elev)
        # the interval just driven belongs to the state we were in
        if self.state == 'climb':
            self.climb_len += ds
            self._climb_pitch_sum += pitch * ds
            if self.climb_len > 0.0:
                self.climb_slope = self._climb_pitch_sum / self.climb_len
        elif self.state == 'deck':
            self.deck_travel += ds

        # back on the floor and level: no longer on a hill
        if elev < self.min_elev and abs(pitch) < self.off:
            self.state = 'flat'
            return self.state

        if self.state == 'flat':
            if pitch > self.on:
                self._start_climb()
            elif pitch < -self.on:
                self.state = 'descent'
        elif self.state == 'climb':
            if abs(pitch) < self.off and elev > self.min_elev:
                self.state = 'deck'
                self.deck_travel = 0.0
        elif self.state == 'deck':
            if pitch > self.on:
                self._start_climb()          # a second climb
            elif pitch < -self.on:
                self.state = 'descent'
        return self.state

    def _start_climb(self):
        self.state = 'climb'
        self.climb_len = 0.0
        self.climb_slope = 0.0
        self._climb_pitch_sum = 0.0

    def deck_ground(self, xs, elev, edge_offset=0.0, deck_len=None):
        """Ground height ahead [m] on the deck, assuming a symmetric hill.

        The deck is as long as deck_len (default: the climb length); its far edge is
        x_edge = deck_len - deck_travel + edge_offset ahead of the robot. Past the edge the
        ground falls at the climb slope until it reaches -elev (the floor), then stays there.
        """
        xs = np.asarray(xs, dtype=float)
        elev = float(elev)
        if deck_len is None:
            deck_len = self.climb_len
        x_edge = float(deck_len) - self.deck_travel + float(edge_offset)
        if self.climb_slope > 0.0:
            z = -np.maximum(0.0, xs - x_edge) * math.tan(self.climb_slope)
        else:
            z = np.where(xs > x_edge, -elev, 0.0)
        return np.maximum(z, -elev)
