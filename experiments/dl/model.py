"""Small U-Net with a stride-2 stem and three skip-connected scales."""

import torch
from torch import nn
from torch.nn import functional as F


def block(inputs, outputs, stride=1):
    return nn.Sequential(
        nn.Conv2d(inputs, outputs, 3, stride=stride, padding=1, bias=False),
        nn.BatchNorm2d(outputs),
        nn.ReLU(inplace=True),
    )


def upsample(feature, skip):
    return torch.cat((F.interpolate(feature, size=skip.shape[-2:], mode="nearest"), skip), dim=1)


class SmallUNet(nn.Module):
    def __init__(self, refine=False):
        super().__init__()
        self.half_res = nn.Sequential(block(3, 12, 2), block(12, 12))
        self.quarter = nn.Sequential(block(12, 20, 2), block(20, 20))
        self.eighth = nn.Sequential(block(20, 32, 2), block(32, 32))
        self.bottleneck = nn.Sequential(block(32, 48, 2), block(48, 48))
        self.up_eighth = nn.Sequential(block(48 + 32, 32), block(32, 24))
        self.up_quarter = nn.Sequential(block(24 + 20, 24), block(24, 16))
        self.up_half = nn.Sequential(block(16 + 12, 16), block(16, 12))
        self.classes = nn.Conv2d(12, 3, 1)
        self.refine = nn.Conv2d(6, 3, 3, padding=1) if refine else None
        if self.refine is not None:
            nn.init.zeros_(self.refine.weight)
            nn.init.zeros_(self.refine.bias)

    def forward(self, image):
        half = self.half_res(image)
        quarter = self.quarter(half)
        eighth = self.eighth(quarter)
        deep = self.bottleneck(eighth)
        decoded = self.up_eighth(upsample(deep, eighth))
        decoded = self.up_quarter(upsample(decoded, quarter))
        decoded = self.up_half(upsample(decoded, half))
        logits = self.classes(decoded)
        if self.refine is not None:
            logits = F.interpolate(logits, size=image.shape[-2:], mode="bilinear", align_corners=False)
            logits = logits + self.refine(torch.cat((image, logits), dim=1))
        return logits
