import unittest

import numpy as np

from experiments.dl.geometry import box, render_mask, rotation


class ProjectionTest(unittest.TestCase):
    def test_closer_obstacle_occludes_line_regardless_of_draw_order(self):
        ribbon = box((4, 0, 0.093), (0.2, 2, 2))
        obstacle = box((2, 0, 0.093), (0.2, 0.1, 0.1))
        mask = render_mask([(obstacle, 2), (ribbon, 1)], np.zeros(3), np.eye(3))
        self.assertEqual(mask.shape, (480, 640))
        self.assertEqual(mask[240, 320], 2)
        self.assertEqual(mask[240, 260], 1)
        self.assertEqual(mask[0, 0], 0)

    def test_quarter_turn_maps_body_x_to_world_y(self):
        np.testing.assert_allclose(rotation(0, 0, np.pi / 2)[:, 0], [0, 1, 0], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
