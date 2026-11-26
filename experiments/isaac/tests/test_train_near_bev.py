import unittest

import torch
from torch.nn import functional as F

from experiments.isaac import train_near


class BevProjectionLossTest(unittest.TestCase):
    def setUp(self):
        torch.manual_seed(1204)
        self.weights = torch.tensor([1., 5.])
        self.logits = torch.randn(2, 2, 240, 320)
        self.predicted = torch.rand(2, 1, 240, 320) * .8 + .3
        self.depth = torch.rand(2, 240, 320) * .8 + .2
        self.line = (torch.rand(2, 240, 320) > .97).long()
        self.valid = (self.line == 1) & (self.depth >= .15) & (self.depth <= 1.4)
        self.bev_mask = torch.zeros(2, train_near.BEV_H, train_near.BEV_W, dtype=torch.uint8)
        self.bev_valid = torch.ones_like(self.bev_mask)

    def test_default_training_loss_is_unchanged(self):
        expected_seg = train_near.segmentation_loss(self.logits, self.line, self.weights)
        errors = F.smooth_l1_loss(self.predicted[:, 0].float(), self.depth, reduction="none", beta=.04)
        expected = expected_seg + 2. * (errors * self.valid).sum() / self.valid.sum().clamp_min(1)
        default = train_near.training_loss(self.logits, self.predicted, self.line, self.depth,
                                           self.valid, self.weights)
        explicit = train_near.training_loss(self.logits, self.predicted, self.line, self.depth,
                                            self.valid, self.weights, bev_weight=0.,
                                            bev_mask=self.bev_mask, bev_valid=self.bev_valid)
        torch.testing.assert_close(default, expected, rtol=0, atol=0)
        torch.testing.assert_close(explicit, expected, rtol=0, atol=0)

    def test_bev_projection_shape_and_probability_range(self):
        projected = train_near.project_to_bev(self.logits, self.predicted)
        self.assertEqual(tuple(projected.shape), (2, train_near.BEV_H, train_near.BEV_W))
        self.assertTrue(bool((projected >= 0).all()))
        self.assertTrue(bool((projected <= 1).all()))

    def test_bev_weight_loss_and_gradients_are_finite(self):
        logits = self.logits.clone().requires_grad_(True)
        predicted = self.predicted.clone().requires_grad_(True)
        with torch.no_grad():
            mask = (train_near.project_to_bev(logits.detach(), predicted.detach()[:, 0]) > .5)
            mask = mask.to(torch.uint8) * 255
        valid = torch.ones_like(mask)
        loss = train_near.training_loss(logits, predicted, self.line, self.depth, self.valid,
                                        self.weights, bev_weight=.5, bev_mask=mask, bev_valid=valid)
        self.assertTrue(torch.isfinite(loss).all())
        loss.backward()
        self.assertTrue(torch.isfinite(logits.grad).all())
        self.assertTrue(torch.isfinite(predicted.grad).all())

    def test_matching_bev_prediction_has_small_loss(self):
        logits = torch.full((1, 2, 240, 320), -10.)
        logits[:, 1, 100:200, 100:120] = 10.
        depth = torch.full((1, 1, 240, 320), .6)
        projected = train_near.project_to_bev(logits, depth)
        mask = (projected > .5).to(torch.uint8) * 255
        valid = torch.ones_like(mask)
        matching = train_near.bev_loss(projected, mask, valid)
        wrong = train_near.bev_loss(torch.zeros_like(projected), mask, valid)
        self.assertLess(float(matching), .01)
        self.assertLess(float(matching), float(wrong))

    def test_background_false_positives_are_penalized(self):
        mask = torch.zeros(1, train_near.BEV_H, train_near.BEV_W, dtype=torch.uint8)
        valid = torch.ones_like(mask)
        quiet = train_near.bev_loss(torch.zeros(1, train_near.BEV_H, train_near.BEV_W), mask, valid)
        noisy = train_near.bev_loss(torch.full((1, train_near.BEV_H, train_near.BEV_W), .2), mask, valid)
        self.assertLess(float(quiet), float(noisy))


if __name__ == "__main__":
    unittest.main()
