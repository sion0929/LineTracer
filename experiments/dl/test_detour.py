import unittest

import numpy as np

from experiments.dl.detour import Box, camera_boxes, choose_detour


class DetourTest(unittest.TestCase):
    def test_chooses_side_away_from_second_box_and_returns_to_line(self):
        blocker = Box(0.51, 0.62, -0.09, 0.10)
        left = Box(0.46, 0.64, 0.22, 0.31)
        path = choose_detour([blocker, left])
        self.assertIsNotNone(path)
        self.assertTrue(np.all(path[:-1, 1] < -0.24))
        self.assertEqual(path[-1, 1], 0)
        self.assertGreater(path[-1, 0], blocker.far)

    def test_rejects_shadow_below_camera_horizon(self):
        mask = np.zeros((480, 640), bool)
        mask[290:390, 245:395] = True
        self.assertEqual(camera_boxes(mask, 0, 0), [])

    def test_vertical_box_estimates_the_bottom_contact(self):
        mask = np.zeros((480, 640), bool)
        mask[208:353, 255:385] = True
        box, = camera_boxes(mask, 0, 0)
        self.assertLess(box.right, 0)
        self.assertGreater(box.left, 0)
        self.assertAlmostEqual(box.near, 0.56, delta=0.04)
