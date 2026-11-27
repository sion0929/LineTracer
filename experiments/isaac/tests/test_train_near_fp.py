import unittest

import torch
from torch.nn import functional as F

from experiments.isaac import train_near


def reference_loss(logits, line, weights):
    probability = logits.float().softmax(1)[:, 1]
    dice = (2 * (probability * line).sum() + 1) / (probability.sum() + line.sum() + 1)
    return F.cross_entropy(logits, line, weight=weights) + .5 * (1 - dice)


class SegmentationLossTest(unittest.TestCase):
    def setUp(self):
        torch.manual_seed(1204)
        self.weights = torch.tensor([1., 5.])
        self.logits = torch.randn(2, 2, 9, 11)
        self.line = (torch.rand(2, 9, 11) > .7).long()

    def test_default_matches_original_formula(self):
        expected = reference_loss(self.logits, self.line, self.weights)
        actual = train_near.segmentation_loss(self.logits, self.line, self.weights)
        torch.testing.assert_close(actual, expected, rtol=0, atol=0)

    def test_false_positive_weight_increases_loss(self):
        logits = torch.zeros(1, 2, 4, 4)
        line = torch.zeros(1, 4, 4, dtype=torch.long)
        line[0, 2, 2] = 1
        logits[0, 1] = 3.
        baseline = train_near.segmentation_loss(logits, line, self.weights)
        penalized = train_near.segmentation_loss(logits, line, self.weights, fp_weight=4.)
        self.assertGreater(float(penalized), float(baseline))

    def test_false_positive_weight_is_inert_without_false_positives(self):
        logits = torch.zeros(1, 2, 4, 4)
        line = torch.zeros(1, 4, 4, dtype=torch.long)
        line[0, 2, 2] = 1
        baseline = train_near.segmentation_loss(logits, line, self.weights)
        penalized = train_near.segmentation_loss(logits, line, self.weights, fp_weight=4.)
        torch.testing.assert_close(penalized, baseline, rtol=1e-6, atol=1e-7)

    def test_ohem_and_boundary_options(self):
        logits = self.logits.clone().requires_grad_(True)
        for kwargs in (dict(ohem_ratio=.25), dict(boundary_band=1),
                       dict(fp_weight=3., ohem_ratio=.25, boundary_band=1)):
            loss = train_near.segmentation_loss(logits, self.line, self.weights, **kwargs)
            self.assertTrue(torch.isfinite(loss).all())
            loss.backward()
            self.assertTrue(torch.isfinite(logits.grad).all())
            logits.grad = None

    def test_ohem_keeps_only_requested_background_share(self):
        loss = torch.arange(16, dtype=torch.float32).reshape(1, 4, 4)
        background = torch.ones(1, 4, 4, dtype=torch.bool)
        keep = train_near.hard_background_keep(loss, background, .25)
        self.assertEqual(int(keep.sum()), 4)
        self.assertTrue(bool(keep.flatten()[12:].all()))


if __name__ == "__main__":
    unittest.main()
