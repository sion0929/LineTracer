import unittest

import numpy as np

from experiments.dl.infer import decode, prepare


class InferenceTest(unittest.TestCase):
    def test_bgr_input_is_reordered_and_normalized(self):
        bgr = np.full((480, 640, 3), (0, 64, 255), dtype=np.uint8)
        batch = prepare(bgr)
        self.assertEqual(batch.shape, (1, 3, 240, 320))
        self.assertEqual(batch.dtype, np.float32)
        np.testing.assert_allclose(batch[0, :, 0, 0], [1, 64 / 255, 0])

    def test_class_masks_are_boolean_and_mutually_exclusive(self):
        scores = np.zeros((1, 3, 2, 2), dtype=np.float32)
        scores[:, 1, :, 0] = 4
        scores[:, 2, :, 1] = 4
        line, obstacle = decode(scores)
        self.assertEqual(line.shape, (480, 640))
        self.assertEqual(line.dtype, np.bool_)
        self.assertEqual(obstacle.dtype, np.bool_)
        self.assertTrue(line[:, :300].all())
        self.assertTrue(obstacle[:, 340:].all())
        self.assertFalse((line & obstacle).any())


if __name__ == "__main__":
    unittest.main()
