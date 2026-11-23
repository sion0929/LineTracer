import unittest

import torch

from experiments.dl.train import apply_shadow


class ShadowAugmentationTest(unittest.TestCase):
    def test_shadow_darkens_one_side_without_brightening_other_pixels(self):
        images = torch.full((1, 3, 9, 11), 0.8)
        center = torch.tensor([0.0])
        slope = torch.tensor([0.0])
        softness = torch.tensor([0.1])
        strength = torch.tensor([0.45])
        result = apply_shadow(images, center, slope, softness, strength)
        torch.testing.assert_close(result[:, :, 0], images[:, :, 0])
        torch.testing.assert_close(result[:, :, -1], images[:, :, -1] * 0.45)
        torch.testing.assert_close(images, torch.full_like(images, 0.8))
        self.assertTrue(torch.all(result <= images))


if __name__ == "__main__":
    unittest.main()
