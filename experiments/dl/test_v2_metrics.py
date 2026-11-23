import unittest

import numpy as np

from experiments.dl.evaluate import hybrid_mask, recall_counts


class V2MetricsTest(unittest.TestCase):
    def test_hybrid_replaces_dark_obstacle_pixels_and_keeps_light_obstacles(self):
        threshold = np.array([[1, 1, 0, 0]], dtype=np.uint8)
        dl = np.array([[0, 2, 2, 1]], dtype=np.uint8)
        np.testing.assert_array_equal(hybrid_mask(threshold, dl), [[1, 2, 2, 0]])

    def test_distance_recall_uses_only_labeled_pixels_in_each_bin(self):
        target = np.array([[0, 1, 1, 1, 2, 2]], dtype=np.uint8)
        prediction = np.array([[0, 1, 0, 1, 2, 2]], dtype=np.uint8)
        depth = np.array([[0.0, 0.2, 0.6, 1.7, 1.6, 4.0]])
        eligible = np.array([[False, True, True, True, True, False]])
        self.assertEqual(
            recall_counts(target, [prediction], depth, 1, eligible).tolist(),
            [[1, 1, 1, 0], [1, 0, 1, 0]],
        )
        self.assertEqual(
            recall_counts(target, [prediction], depth, 2, eligible).tolist(),
            [[0, 0, 1, 0], [0, 0, 1, 0]],
        )
