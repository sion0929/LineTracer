import unittest

import numpy as np

from racing_line import nearest_station, plan_path, speed_ceiling


LIMITS = dict(vmax=0.38, lateral_accel=0.30, accel=0.45, braking=0.60, wmax=1.4, initial_speed=0.20)


class RacingLineTest(unittest.TestCase):
    def test_corner_line_improves_time_inside_centerline_corridor(self):
        approach = np.column_stack((np.linspace(0.0, 0.29, 30), np.zeros(30)))
        angle = np.linspace(-np.pi / 2, 0.0, 21)
        corner = np.column_stack((0.30 + 0.10 * np.cos(angle), 0.10 + 0.10 * np.sin(angle)))
        exit_line = np.column_stack((np.full(35, 0.40), np.linspace(0.11, 0.45, 35)))
        centerline = np.vstack((approach, corner, exit_line))

        optimized = plan_path(centerline, **LIMITS)
        reference = speed_ceiling(centerline, **LIMITS)

        self.assertLess(optimized.travel_time, reference.travel_time)
        self.assertLessEqual(np.max(np.linalg.norm(optimized.points - centerline, axis=1)), 0.10 + 1e-9)
        np.testing.assert_array_equal(optimized.points[[0, -1]], centerline[[0, -1]])

    def test_speed_profile_cruises_and_can_brake_to_end(self):
        centerline = np.column_stack((np.linspace(0.0, 0.8, 81), np.zeros(81)))
        reaction_time = 0.08
        profile = speed_ceiling(centerline, reaction_time=reaction_time, **LIMITS)
        squared = profile.speed ** 2
        braking_room = 2 * LIMITS["braking"] * np.diff(profile.arc)
        stopping_distance = reaction_time * profile.speed + squared / (2 * LIMITS["braking"])

        self.assertEqual(np.max(profile.speed), LIMITS["vmax"])
        self.assertEqual(profile.speed[-1], 0.0)
        self.assertTrue(np.all(squared[:-1] <= squared[1:] + braking_room + 1e-9))
        self.assertTrue(np.all(stopping_distance <= profile.arc[-1] - profile.arc + 1e-9))

    def test_projection_does_not_jump_to_return_lane(self):
        centerline = np.array([[0.0, 0.0], [0.20, 0.0], [0.40, 0.0],
                               [0.40, 0.04], [0.10, 0.04]])
        profile = speed_ceiling(centerline, **LIMITS)

        self.assertAlmostEqual(nearest_station(profile, (0.12, 0.039)), 0.12)


if __name__ == "__main__":
    unittest.main()
