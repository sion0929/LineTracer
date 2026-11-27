#!/usr/bin/env python3
"""Train the line-restoration U-Net (seen line + seen area -> route), export ONNX.

Metrics on the validation set (unseen tracks: hills60 + case suite):
  route IoU        predicted route vs true route (all route pixels)
  unseen recall    share of true route pixels never seen by the camera that the model restores
                   (within 2 px = 2.5 cm)
  false rate       predicted route pixels farther than 2 px from the true route
  baseline         the same numbers for "copy the seen line" (no restoration)

  python experiments/restore/train.py --train train.npz --val val.npz --out runs/r1 --epochs 30
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


class Block(nn.Module):
    def __init__(self, i, o):
        super().__init__()
        self.c = nn.Sequential(nn.Conv2d(i, o, 3, padding=1, bias=False), nn.BatchNorm2d(o), nn.ReLU(inplace=True),
                               nn.Conv2d(o, o, 3, padding=1, bias=False), nn.BatchNorm2d(o), nn.ReLU(inplace=True))

    def forward(self, x):
        return self.c(x)


class UNet(nn.Module):
    def __init__(self, ch=(16, 32, 64, 96, 128)):
        super().__init__()
        self.enc = nn.ModuleList()
        prev = 2
        for c in ch:
            self.enc.append(Block(prev, c))
            prev = c
        self.dec = nn.ModuleList()
        for c in reversed(ch[:-1]):
            self.dec.append(Block(prev + c, c))
            prev = c
        self.head = nn.Conv2d(prev, 1, 1)

    def forward(self, x):
        skips = []
        for i, b in enumerate(self.enc):
            x = b(x)
            if i < len(self.enc) - 1:
                skips.append(x)
                x = F.max_pool2d(x, 2)
        for b, s in zip(self.dec, reversed(skips)):
            x = F.interpolate(x, scale_factor=2, mode="bilinear", align_corners=False)
            x = b(torch.cat([x, s], 1))
        return self.head(x)


def dilate(m, r=2):
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
    return np.stack([cv2.dilate(a, k) for a in m])


def metrics(pred, y, unseen):
    y_d, p_d = dilate(y), dilate(pred)
    inter = (pred & y).sum()
    iou = inter / max(1, (pred | y).sum())
    unseen_rec = (p_d & unseen).sum() / max(1, unseen.sum())
    false = (pred & (1 - y_d)).sum() / max(1, pred.sum())
    route_rec = (p_d & y).sum() / max(1, y.sum())
    return dict(route_iou=float(iou), route_recall=float(route_rec), unseen_recall=float(unseen_rec),
                false_rate=float(false))


def load_dataset(path):
    with np.load(path) as archive:
        return dict(archive)


def frontier_loss_weight(x, y, boost, hard_negative):
    unseen = 1.0 - x[:, 1:2]
    seeds = x[:, :1] * F.max_pool2d(unseen, 5, stride=1, padding=2)
    # Pixel centers above x = 0.2 m occupy the first 80 rows.
    seeds[:, :, 80:] = 0
    # Half-width 32 cells = 0.4 m. The square window is a training proxy
    # for the evaluation radius and includes all candidate frontier pixels.
    near = F.max_pool2d(seeds, (65, 1), stride=1, padding=(32, 0))
    near = F.max_pool2d(near, (1, 65), stride=1, padding=(0, 32)) * unseen
    return near * (boost * y + hard_negative * (1.0 - y))


def export_onnx(net, path):
    net.eval().cpu()
    torch.onnx.export(net, torch.zeros(1, 2, 128, 128), path, input_names=["x"],
                      output_names=["logit"], dynamic_axes={"x": {0: "batch"}, "logit": {0: "batch"}},
                      opset_version=17, dynamo=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--train", required=True, help="one or more .npz, comma separated")
    ap.add_argument("--val", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--lr", type=float, default=2e-3)
    ap.add_argument("--width", type=int, default=16)
    ap.add_argument("--unseen-weight", type=float, default=3.0, help="extra loss weight on unseen route pixels")
    ap.add_argument("--frontier-boost", type=float, default=0.0)
    ap.add_argument("--frontier-negative", type=float, default=0.0,
                    help="extra focal loss on off-route predictions near a frontier")
    ap.add_argument("--init", type=Path, help="warm start from a checkpoint of the same width")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--amp", action="store_true", help="use CUDA mixed precision")
    a = ap.parse_args()
    if not torch.cuda.is_available():
        ap.error("CUDA is unavailable")
    torch.set_num_threads(4)
    cv2.setNumThreads(4)
    torch.manual_seed(a.seed)
    rng = np.random.default_rng(a.seed)
    a.out.mkdir(parents=True, exist_ok=True)
    parts = [load_dataset(f) for f in a.train.split(",")]
    va = load_dataset(a.val)
    Xt = np.concatenate([t["x"] for t in parts])
    Yt = np.concatenate([t["y"] for t in parts])
    Ut = np.concatenate([t["unseen"] for t in parts])
    del parts
    Xv, Yv, Uv = va["x"], va["y"], va["unseen"]
    dev = "cuda"
    w = a.width
    net = UNet((w, 2 * w, 4 * w, 6 * w, 8 * w)).to(dev)
    if a.init:
        net.load_state_dict(torch.load(a.init, map_location=dev, weights_only=True))
    scaler = torch.amp.GradScaler("cuda", enabled=a.amp)
    opt = torch.optim.AdamW(net.parameters(), lr=a.lr, weight_decay=1e-4)
    steps = len(Xt) // a.batch
    if not steps:
        ap.error("--batch exceeds the training set size")
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=a.lr, epochs=a.epochs, steps_per_epoch=steps)
    pos_w = torch.tensor(8.0, device=dev)
    base = metrics(Xv[:, 0].astype(np.uint8), Yv, Uv)
    print(json.dumps({"baseline_copy_seen": base, "train": len(Xt), "val": len(Xv),
                      "params": sum(p.numel() for p in net.parameters())}), flush=True)
    best, hist = -1, []
    for ep in range(a.epochs):
        net.train()
        perm = rng.permutation(len(Xt))
        t0 = time.time()
        tot = torch.zeros((), device=dev)
        for i in range(0, len(perm) - a.batch + 1, a.batch):
            idx = np.sort(perm[i:i + a.batch])
            x = torch.from_numpy(Xt[idx]).float().to(dev)
            y = torch.from_numpy(Yt[idx]).float().unsqueeze(1).to(dev)
            u = torch.from_numpy(Ut[idx]).float().unsqueeze(1).to(dev)
            if rng.random() < 0.5:                          # left/right mirror
                x, y, u = x.flip(-1), y.flip(-1), u.flip(-1)
            with torch.autocast("cuda", enabled=a.amp):
                logit = net(x)
            logit = logit.float()
            wmap = 1.0 + a.unseen_weight * u
            bce = F.binary_cross_entropy_with_logits(logit, y, pos_weight=pos_w, weight=wmap)
            p = torch.sigmoid(logit)
            if a.frontier_boost or a.frontier_negative:
                weight = frontier_loss_weight(x, y, a.frontier_boost, a.frontier_negative)
                focal = y + (1.0 - y) * p.detach().square()
                bce += (F.binary_cross_entropy_with_logits(logit, y, reduction="none",
                                                           pos_weight=pos_w) * weight * focal).mean()
            dice = 1 - (2 * (p * y).sum((1, 2, 3)) + 1) / (p.sum((1, 2, 3)) + y.sum((1, 2, 3)) + 1)
            loss = bce + dice.mean()
            opt.zero_grad(set_to_none=True)
            scaler.scale(loss).backward()
            scaler.step(opt)
            scaler.update()
            sched.step()
            tot += loss.detach()
        net.eval()
        preds = []
        with torch.no_grad():
            for i in range(0, len(Xv), a.batch):
                x = torch.from_numpy(Xv[i:i + a.batch]).float().to(dev)
                preds.append((torch.sigmoid(net(x))[:, 0] > 0.5).cpu().numpy().astype(np.uint8))
        m = metrics(np.concatenate(preds), Yv, Uv)
        m.update(epoch=ep + 1, loss=float(tot) / steps, sec=round(time.time() - t0, 1))
        hist.append(m)
        print(json.dumps(m), flush=True)
        score = m["route_iou"] + m["unseen_recall"] - m["false_rate"]
        if score > best:
            best = score
            torch.save(net.state_dict(), a.out / "best.pt")
        (a.out / "metrics.json").write_text(json.dumps(
            {"baseline_copy_seen": base, "history": hist, "config": vars(a)}, indent=1, default=str))
    net.load_state_dict(torch.load(a.out / "best.pt", map_location=dev, weights_only=True))
    export_onnx(net, a.out / "restore.onnx")
    print("saved", a.out / "restore.onnx")


if __name__ == "__main__":
    main()
