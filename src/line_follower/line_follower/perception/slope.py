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
        self.rise = 0.0               # height gained over the climb [m]
        self.up_len = 0.0             # ramp length from rise / tan(slope) [m]
        self._pitches = []            # pitch samples while climbing
        self._elev0 = 0.0             # floor height the climb started from
        self.pre_len = 0.0            # ramp already climbed when the climb was noticed [m]
        self._elev_notice = 0.0

    def update(self, ds, pitch, elev):
        """Advance by ds [m] with the current pitch [rad] and elevation [m]; returns the state."""
        ds = max(0.0, float(ds))
        pitch = float(pitch)
        elev = float(elev)
        # the interval just driven belongs to the state we were in
        if self.state == 'climb':
            if self._elev_notice is None:
                self._elev_notice = elev
            self.climb_len += ds
            if ds > 0.0:
                self._pitches.append(pitch)
            # The body pitch rises and falls over the wheelbase at both ends of the ramp, so the
            # mean pitch under-reads the slope and the 'climb' span over-reads the ramp. The upper
            # percentile is the slope; the height gained divided by tan(slope) is the ramp length.
            if self._pitches:
                self.climb_slope = float(np.percentile(self._pitches, 90))
            self.rise = max(0.0, elev - self._elev0)
        elif self.state == 'deck':
            self.deck_travel += ds

        # back on the floor and level: no longer on a hill
        if elev < self.min_elev and abs(pitch) < self.off:
            self.state = 'flat'
            return self.state

        if self.state == 'flat':
            if pitch > self.on:
                self._start_climb(0.0)       # from the floor: the rise counts from z = 0
            elif pitch < -self.on:
                self.state = 'descent'
        elif self.state == 'climb':
            if abs(pitch) < self.off and elev > self.min_elev:
                self.state = 'deck'
                self.deck_travel = 0.0
                self.rise = max(self.rise, elev - self._elev0)
                self.up_len = (self.rise / math.tan(self.climb_slope)
                               if self.climb_slope > math.radians(1.0) else self.climb_len)
                self.up_len = min(self.up_len, self.climb_len + 0.1)
                if self.climb_slope > math.radians(1.0) and self._elev_notice is not None:
                    self.pre_len = max(0.0, self._elev_notice - self._elev0) / math.tan(self.climb_slope)
        elif self.state == 'deck':
            if pitch > self.on:
                self._start_climb(elev)      # a second climb
            elif pitch < -self.on:
                self.state = 'descent'
        return self.state

    def _start_climb(self, elev=0.0):
        self.state = 'climb'
        self.climb_len = 0.0
        self.climb_slope = 0.0
        self.rise = 0.0
        self.up_len = 0.0
        self._pitches = []
        self._elev0 = float(elev)
        self.pre_len = 0.0
        self._elev_notice = None

    def deck_ground(self, xs, elev, edge_offset=0.0, deck_len=None):
        """Ground height ahead [m] on the deck, assuming a symmetric hill.

        The deck is as long as deck_len (default: the up-ramp length). The robot noticed the deck
        `lag` = pre_len + climb_len - up_len after it really began (pitch rises and settles over
        the wheelbase; pre_len is the ramp already climbed when the climb was noticed), so the
        far edge is x_edge = deck_len - lag - deck_travel + edge_offset ahead of the robot. Past the
        edge the ground falls at the climb slope until it reaches -elev (the floor).
        """
        xs = np.asarray(xs, dtype=float)
        elev = float(elev)
        up = self.up_len if self.up_len > 0.0 else self.climb_len
        if deck_len is None:
            deck_len = up
        lag = max(0.0, self.pre_len + self.climb_len - up)
        x_edge = float(deck_len) - lag - self.deck_travel + float(edge_offset)
        if self.climb_slope > 0.0:
            z = -np.maximum(0.0, xs - x_edge) * math.tan(self.climb_slope)
        else:
            z = np.where(xs > x_edge, -elev, 0.0)
        return np.maximum(z, -elev)
