#!/usr/bin/env python3
"""Train the far-band line model: 64x640 full-resolution horizon strip -> line logits (dev).

Data: experiments/farband/capture.py npz files (x: N,64,640,3 RGB, y: N,64,640 line coverage 0-255).
Split by map (last --val-maps maps). Reports pixel IoU and recall by distance band, next to the
rule threshold on the same strips, and exports ONNX (input "rgb" 1x3x64x640 /255, output "logits").
  /home/rvlab/miniforge3/envs/lf_b_dl/bin/python experiments/farband/train.py --data DIR --out RUN
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/line_follower"))
from line_follower import vision  # noqa: E402

ROW0 = 256


def row_dist(r):
    """forward distance from the axle of strip row r (flat ground)."""
    v = ROW0 + r
    return vision.FX * vision.CAM_H / (v - vision.CY) + vision.CAM_X


class Block(nn.Module):
    def __init__(self, i, o):
        super().__init__()
        self.c = nn.Sequential(nn.Conv2d(i, o, 3, padding=1, bias=False), nn.BatchNorm2d(o), nn.ReLU(inplace=True),
                               nn.Conv2d(o, o, 3, padding=1, bias=False), nn.BatchNorm2d(o), nn.ReLU(inplace=True))

    def forward(self, x):
        return self.c(x)


class BandNet(nn.Module):
    """Small U-Net, 3 levels. Pools are 2x2 (64x640 -> 8x80)."""

    def __init__(self, ch=(16, 32, 48, 64)):
        super().__init__()
        self.enc = nn.ModuleList([Block(3, ch[0])] + [Block(ch[k], ch[k + 1]) for k in range(len(ch) - 1)])
        self.dec = nn.ModuleList([Block(ch[k + 1] + ch[k], ch[k]) for k in reversed(range(len(ch) - 1))])
        self.head = nn.Conv2d(ch[0], 1, 1)

    def forward(self, x):
        skips = []
        for k, e in enumerate(self.enc):
            x = e(x if k == 0 else F.max_pool2d(x, 2))
            skips.append(x)
        skips.pop()
        for d in self.dec:
            s = skips.pop()
            x = d(torch.cat([F.interpolate(x, size=s.shape[-2:], mode="bilinear", align_corners=False), s], 1))
        return self.head(x)


def load(files, hscale=1):
    """Read strips, downscaling width per file (a whole-set float64 mean needs ~7 GB and OOMs the host)."""
    X, Y = [], []
    for f in files:
        z = np.load(f)
        x, y = z["x"], z["y"]
        if x.ndim != 4 or not len(x):
            continue
        if hscale > 1:
            x = x.reshape(*x.shape[:2], -1, hscale, 3).mean(3, dtype=np.float32).round().astype(np.uint8)
            y = y.reshape(*y.shape[:2], -1, hscale).max(3)
        X.append(x); Y.append(y)
    return np.concatenate(X), np.concatenate(Y)


def augment(x, y, rng):
    """x: B,64,640,3 uint8, y: B,64,640 uint8 -> float tensors with photometric + geometric jitter."""
    x = x.astype(np.float32) / 255.0
    y = y.astype(np.float32) / 255.0
    B = len(x)
    flip = rng.random(B) < 0.5
    x[flip] = x[flip, :, ::-1]; y[flip] = y[flip, :, ::-1]
    for i in range(B):
        sh = int(rng.integers(-8, 9))                     # pitch jitter: the band moves vertically
        if sh:
            x[i] = np.roll(x[i], sh, 0); y[i] = np.roll(y[i], sh, 0)
            if sh > 0:
                x[i, :sh] = x[i, sh:sh + 1]; y[i, :sh] = 0
            else:
                x[i, sh:] = x[i, sh - 1:sh]; y[i, sh:] = 0
        if rng.random() < 0.3:
            k = int(rng.choice([3, 5]))
            x[i] = cv2.GaussianBlur(x[i], (k, 1 if rng.random() < 0.5 else 3), 0)
    gain = rng.uniform(0.6, 1.4, (B, 1, 1, 1)) * rng.uniform(0.85, 1.15, (B, 1, 1, 3))
    bias = rng.uniform(-0.15, 0.15, (B, 1, 1, 1))
    gamma = rng.uniform(0.7, 1.4, (B, 1, 1, 1))
    x = np.clip(np.clip(x * gain + bias, 0, 1) ** gamma + rng.normal(0, rng.uniform(0, 0.04), x.shape), 0, 1)
    return torch.from_numpy(np.ascontiguousarray(x)).permute(0, 3, 1, 2).float(), torch.from_numpy(np.ascontiguousarray(y))[:, None]


def augment_gpu(xb, yb, g):
    """Same jitter as augment(), on the GPU. xb: B,H,W,3 uint8 cuda, yb: B,H,W uint8 cuda."""
    B, H, W, _ = xb.shape
    dev = xb.device
    x = xb.permute(0, 3, 1, 2).float() / 255.0
    y = (yb.float() / 255.0)[:, None]
    flip = torch.rand(B, device=dev, generator=g) < 0.5
    x = torch.where(flip[:, None, None, None], x.flip(-1), x)
    y = torch.where(flip[:, None, None, None], y.flip(-1), y)
    sh = torch.randint(-8, 9, (B,), device=dev, generator=g)          # pitch jitter
    rows = torch.arange(H, device=dev)[None] - sh[:, None]              # source row per output row
    inside = (rows >= 0) & (rows < H)
    rows_c = rows.clamp(0, H - 1)
    x = torch.gather(x, 2, rows_c[:, None, :, None].expand(B, 3, H, W))
    y = torch.gather(y, 2, rows_c[:, None, :, None].expand(B, 1, H, W)) * inside[:, None, :, None]
    blur = torch.rand(B, device=dev, generator=g) < 0.3
    if blur.any():
        k = torch.tensor([1., 4., 6., 4., 1.], device=dev); k = k / k.sum()
        kv = torch.tensor([1., 2., 1.], device=dev); kv = kv / kv.sum()
        ker = (kv[:, None] * k[None, :])[None, None].repeat(3, 1, 1, 1)
        xb_ = F.conv2d(F.pad(x[blur], (2, 2, 1, 1), mode="replicate"), ker, groups=3)
        x = x.clone(); x[blur] = xb_
    u = lambda lo, hi, *s: lo + (hi - lo) * torch.rand(*s, device=dev, generator=g)  # noqa: E731
    gain = u(0.6, 1.4, B, 1, 1, 1) * u(0.85, 1.15, B, 3, 1, 1)
    bias = u(-0.15, 0.15, B, 1, 1, 1)
    gamma = u(0.7, 1.4, B, 1, 1, 1)
    noise = torch.randn(x.shape, device=dev, generator=g) * u(0.0, 0.04, B, 1, 1, 1)
    x = ((x * gain + bias).clamp(0, 1) ** gamma + noise).clamp(0, 1)
    return x, y


def rule_mask(x):
    g = cv2.cvtColor(x, cv2.COLOR_RGB2GRAY)
    t = min(125.0, 0.62 * float(np.median(g)))
    return g < t


def scores(pred, y, rows_d):
    """pred, y: bool arrays N,64,640. IoU, and recall per distance band (thin lines included)."""
    inter = (pred & y).sum(); union = (pred | y).sum()
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    pd = np.array([cv2.dilate(p.astype(np.uint8), k) for p in pred]).astype(bool)
    out = {"iou": float(inter / max(1, union)), "precision": float((pred & y).sum() / max(1, pred.sum()))}
    for lo, hi in ((0.8, 1.0), (1.0, 1.2), (1.2, 1.5), (1.5, 2.0)):
        rr = (rows_d >= lo) & (rows_d < hi)
        yy = y[:, rr]; out[f"recall_{lo}-{hi}"] = float((pd[:, rr] & yy).sum() / max(1, yy.sum()))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="comma-separated capture folders (flat, hills, ...)")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--val-maps", type=int, default=10)
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--batch", type=int, default=48)
    ap.add_argument("--lr", type=float, default=2e-3)
    ap.add_argument("--width", type=int, default=16)
    ap.add_argument("--hscale", type=int, default=1, help="downscale columns by this factor (rows stay full res)")
    ap.add_argument("--gpu-aug", action="store_true", help="keep the training set on the GPU and augment there")
    ap.add_argument("--target", type=float, default=0.2, help="line where coverage > target (binary)")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    files = sorted(f for d in a.data.split(",") for f in Path(d).glob("*.npz"))
    names = sorted({f.stem for f in files})
    val_names = set(names[-a.val_maps:])              # same held-out maps in every folder
    tr = [f for f in files if f.stem not in val_names]
    va = [f for f in files if f.stem in val_names]
    Xt, Yt = load(tr, a.hscale); Xv, Yv = load(va, a.hscale)
    Yt = ((Yt > a.target * 255) * 255).astype(np.uint8)
    print(f"train {len(Xt)} strips ({len(tr)} maps), val {len(Xv)} ({len(va)} maps)", flush=True)
    rows_d = np.array([row_dist(r) for r in range(Xt.shape[1])])
    yv = Yv > a.target * 255
    base = scores(np.array([rule_mask(x) for x in Xv]), yv, rows_d)
    print("rule threshold on val:", json.dumps({k: round(v, 3) for k, v in base.items()}), flush=True)
    w = a.width
    net = BandNet((w, 2 * w, 3 * w, 4 * w)).cuda()
    opt = torch.optim.AdamW(net.parameters(), lr=a.lr, weight_decay=1e-4)
    steps = a.epochs * math.ceil(len(Xt) / a.batch)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, a.lr, total_steps=steps)
    rng = np.random.default_rng(0)
    best, hist = -1, []
    for ep in range(a.epochs):
        net.train(); t0 = time.time(); perm = rng.permutation(len(Xt)); tl = 0
        for i in range(0, len(perm), a.batch):
            idx = perm[i:i + a.batch]
            if a.gpu_aug:
                if ep == 0 and i == 0:
                    Xg, Yg = torch.from_numpy(Xt).cuda(), torch.from_numpy(Yt).cuda()
                    gen = torch.Generator(device="cuda"); gen.manual_seed(0)
                ii = torch.from_numpy(idx).cuda()
                x, y = augment_gpu(Xg[ii], Yg[ii], gen)
            else:
                x, y = augment(Xt[idx], Yt[idx], rng)
                x, y = x.cuda(), y.cuda()
            lg = net(x)
            bce = F.binary_cross_entropy_with_logits(lg, y, pos_weight=torch.tensor(4.0, device="cuda"))
            p = torch.sigmoid(lg)
            dice = 1 - (2 * (p * y).sum() + 1) / (p.sum() + y.sum() + 1)
            loss = bce + dice
            opt.zero_grad(); loss.backward(); opt.step(); sched.step(); tl += loss.item() * len(idx)
        net.eval(); P = []
        with torch.no_grad():
            for i in range(0, len(Xv), 128):
                x = torch.from_numpy(Xv[i:i + 128]).permute(0, 3, 1, 2).float().cuda() / 255
                P.append((net(x)[:, 0] > 0).cpu().numpy())
        s = scores(np.concatenate(P), yv, rows_d)
        hist.append(dict(epoch=ep, loss=tl / len(Xt), **s))
        print(f"ep {ep} loss {tl/len(Xt):.4f} " + " ".join(f"{k} {v:.3f}" for k, v in s.items()) + f"  {time.time()-t0:.0f}s", flush=True)
        far = (s["recall_1.0-1.2"] + s["recall_1.2-1.5"]) / 2
        sel = 2 * far * s["precision"] / max(1e-6, far + s["precision"])     # far recall x precision
        if sel > best:
            best = sel; torch.save(net.state_dict(), a.out / "best.pt")
    net.load_state_dict(torch.load(a.out / "best.pt")); net.eval().cpu()
    torch.onnx.export(net, torch.zeros(1, 3, 64, 640 // a.hscale), a.out / "farband.onnx", input_names=["rgb"],
                      output_names=["logits"], opset_version=17, dynamo=False)
    json.dump(dict(rule=base, history=hist, best_sel=best, width=w, hscale=a.hscale, target=a.target),
              open(a.out / "metrics.json", "w"), indent=1)
    json.dump(dict(width=640 // a.hscale, rows=64), open(a.out / "farband.json", "w"))
    print("saved", a.out / "farband.onnx", "best iou", round(best, 3))


if __name__ == "__main__":
    main()
