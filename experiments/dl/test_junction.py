import unittest

import numpy as np

from experiments.dl.junction import terminal_shape


def segment(start, end):
    return np.linspace(start, end, 40)


class JunctionTest(unittest.TestCase):
    def test_terminal_is_compact_and_has_no_forward_branch(self):
        points = np.vstack((segment((-0.15, 0), (0, 0)), segment((0, -0.10), (0, 0.10))))
        terminal, forward = terminal_shape(points)
        self.assertTrue(terminal)
        self.assertIsNone(forward)

    def test_partial_crossing_can_reconnect_to_forward_line_across_a_gap(self):
        points = np.vstack((segment((-0.15, 0), (0, 0)), segment((0, -0.18), (0, 0.17)),
                            segment((0.15, 0.018), (0.24, 0.02))))
        terminal, forward = terminal_shape(points)
        self.assertFalse(terminal)
        self.assertTrue(0.14 < forward[0] < 0.22)

    def test_long_arms_are_not_a_terminal(self):
        points = np.vstack((segment((-0.15, 0), (0, 0)), segment((0, -0.18), (0, 0.17))))
        self.assertEqual(terminal_shape(points), (False, None))
