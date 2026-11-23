import math
import unittest

import numpy as np

from experiments.dl.corridor import guarded_speed, recovery_motion_supported, wheel_aware_path, wheel_risk


class CorridorTest(unittest.TestCase):
    def test_recovery_cannot_drive_straight_past_the_corner(self):
        stations = np.arange(0.0, 0.21, 0.01)
        mapped = np.vstack((np.column_stack((-stations, 0 * stations)),
                            np.column_stack((0 * stations, stations))))
        self.assertFalse(recovery_motion_supported(mapped, (-0.01, 0), 0))
        self.assertTrue(recovery_motion_supported(mapped, (-0.01, 0), math.pi / 2))

    def test_slow_for_tight_bend_ahead_but_keep_straight_speed(self):
        x = np.arange(0.0, 0.41, 0.01)
        straight = np.column_stack((x, x * 0))
        bend = np.vstack((straight[:14], np.column_stack((np.full(14, 0.13), x[1:15]))))
        self.assertAlmostEqual(guarded_speed(straight, 0.1, 0.0), 0.1)
        self.assertLessEqual(guarded_speed(bend, 0.1, 0.0), 0.055)

    def test_stop_translation_when_bend_target_requires_rotation(self):
        arc = np.linspace(0, math.pi / 2, 24)
        bend = np.column_stack((0.11 * np.sin(arc), 0.11 * (1 - np.cos(arc))))
        self.assertEqual(guarded_speed(bend, 0.10, 0.80), 0.0)

    def test_unchanged_straight_and_always_within_corridor(self):
        x = np.linspace(0, 0.30, 31)
        straight = np.column_stack((x, np.zeros_like(x)))
        np.testing.assert_array_equal(wheel_aware_path(straight, 0.10), straight)
        arc = np.linspace(0, math.pi / 2, 30)
        bend = np.column_stack((0.11 * np.sin(arc), 0.11 * (1 - np.cos(arc))))
        candidate = wheel_aware_path(bend, 0.10)
        self.assertLessEqual(np.linalg.norm(candidate - bend, axis=1).max(), 0.10)
        self.assertLessEqual(wheel_risk(candidate, bend), wheel_risk(bend, bend))
