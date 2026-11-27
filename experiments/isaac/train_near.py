#!/usr/bin/env python3
"""Train the line-only V4 segmentation and metric-depth model."""
import argparse
import json
import math
from itertools import islice
from pathlib import Path

import cv2
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F
from torch.utils.data import ConcatDataset, DataLoader, Dataset

if __package__:
    from .make_farband_from_isaac import DATASET, npz_frames
else:
    from make_farband_from_isaac import DATASET, npz_frames

SIZE = (320, 240)

# Camera and BEV geometry, copied from src/line_follower/line_follower/{vision,depth}.py.
# The Isaac archives store bev_mask/bev_valid on this exact 108x220 grid.
CAMERA_W, CAMERA_H = 640, 480
HFOV = 1.085595
FX = (CAMERA_W / 2.0) / math.tan(HFOV / 2.0)
CX, CY = (CAMERA_W - 1) / 2.0, (CAMERA_H - 1) / 2.0
CAM_X = 0.076
REST_PITCH = math.radians(0.33)
BEV_RES = 0.005
BEV_X_NEAR = 0.31
BEV_X_FAR = 0.85
BEV_Y_HALF = 0.55
BEV_W = int(round(2 * BEV_Y_HALF / BEV_RES))
BEV_H = int(round((BEV_X_FAR - BEV_X_NEAR) / BEV_RES))


def block(inputs, outputs, stride=1):
    return nn.Sequential(
        nn.Conv2d(inputs, outputs, 3, stride=stride, padding=1, bias=False),
        nn.BatchNorm2d(outputs),
        nn.ReLU(inplace=True),
    )


def upsample(feature, skip):
    return torch.cat((F.interpolate(feature, size=skip.shape[-2:], mode="nearest"), skip), dim=1)


def splat_bev(probability, depth):
    """Differentiably project per-pixel line probability to the robot BEV grid.

    `probability` is (B, H, W) and `depth` is (B, H, W) or (B, 1, H, W) metric
    camera-axis depth. Each pixel is bilinearly splatted at the BEV cell that
    depth.py would pick (x forward, y left); out-of-grid contributions are
    dropped and the accumulated probability is clamped to [0, 1].
    """
    probability = probability.float()
    depth = depth.float()
    if probability.dim() == 4:
        probability = probability[:, 0]
    if depth.dim() == 4:
        depth = depth[:, 0]
    if probability.dim() == 2:
        probability = probability[None]
    if depth.dim() == 2:
        depth = depth[None]
    device, dtype = probability.device, probability.dtype
    u = (torch.arange(SIZE[0], device=device, dtype=dtype) + .5) * (CAMERA_W / SIZE[0]) - .5
    v = (torch.arange(SIZE[1], device=device, dtype=dtype) + .5) * (CAMERA_H / SIZE[1]) - .5
    ru = (u - CX) / FX
    rv = (v - CY) / FX
    down = rv[None, :, None] * depth
    x = CAM_X + depth * math.cos(REST_PITCH) + down * math.sin(REST_PITCH)
    y = -ru[None, None, :] * depth
    columns = (BEV_Y_HALF - y) / BEV_RES
    rows = (BEV_X_FAR - x) / BEV_RES
    row0, column0 = torch.floor(rows), torch.floor(columns)
    row_fraction, column_fraction = rows - row0, columns - column0
    row0, column0 = row0.long(), column0.long()
    row1, column1 = row0 + 1, column0 + 1
    weights = ((1 - row_fraction) * (1 - column_fraction),
               (1 - row_fraction) * column_fraction,
               row_fraction * (1 - column_fraction),
               row_fraction * column_fraction)
    base = torch.arange(probability.shape[0], device=device).view(-1, 1, 1) * (BEV_H * BEV_W)
    flat = torch.zeros(probability.shape[0] * BEV_H * BEV_W, device=device, dtype=dtype)
    for row, column, weight in zip((row0, row0, row1, row1),
                                   (column0, column1, column0, column1), weights):
        inside = (row >= 0) & (row < BEV_H) & (column >= 0) & (column < BEV_W)
        index = (base + row * BEV_W + column)[inside]
        flat.scatter_add_(0, index, (probability * weight)[inside])
    return flat.view(probability.shape[0], BEV_H, BEV_W).clamp(0., 1.)


def project_to_bev(logits, depth):
    """Line logits and metric depth -> differentiable BEV line probability."""
    return splat_bev(logits.float().softmax(1)[:, 1], depth)


def bev_loss(probability, mask, valid):
    """BCE over valid BEV cells; every background cell is scored too."""
    valid = valid > 0
    if not bool(valid.any()):
        return probability.sum() * 0
    target = (mask > 127).float()
    with torch.autocast(device_type=probability.device.type, enabled=False):   # BCE is unsafe under AMP
        p = probability.float()[valid].clamp(1e-6, 1 - 1e-6)
        return F.binary_cross_entropy(p, target[valid])


class V4(nn.Module):
    """b-dlseg V4 with the learned sigmoid depth head and identical parameter names."""

    def __init__(self, small):
        super().__init__()
        c1, c2, c3, c4 = (8, 12, 20, 32) if small else (12, 20, 32, 48)
        d3, d2 = 3 * c3 // 4, 4 * c2 // 5
        self.half_res = nn.Sequential(block(3, c1, 2), block(c1, c1))
        self.quarter = nn.Sequential(block(c1, c2, 2), block(c2, c2))
        self.eighth = nn.Sequential(block(c2, c3, 2), block(c3, c3))
        self.bottleneck = nn.Sequential(block(c3, c4, 2), block(c4, c4))
        self.up_eighth = nn.Sequential(block(c4 + c3, c3), block(c3, d3))
        self.up_quarter = nn.Sequential(block(d3 + c2, d3), block(d3, d2))
        self.up_half = nn.Sequential(block(d2 + c1, d2), block(d2, c1))
        self.classes = nn.Conv2d(c1, 2, 1)
        self.refine = nn.Conv2d(5, 2, 3, padding=1)
        nn.init.zeros_(self.refine.weight)
        nn.init.zeros_(self.refine.bias)
        self.depth = nn.Conv2d(c1, 1, 1)

    def forward(self, image):
        half = self.half_res(image)
        quarter = self.quarter(half)
        eighth = self.eighth(quarter)
        deep = self.bottleneck(eighth)
        decoded = self.up_eighth(upsample(deep, eighth))
        decoded = self.up_quarter(upsample(decoded, quarter))
        decoded = self.up_half(upsample(decoded, half))
        logits = F.interpolate(self.classes(decoded), size=SIZE[::-1], mode="bilinear", align_corners=False)
        depth = F.interpolate(self.depth(decoded), size=SIZE[::-1], mode="bilinear", align_corners=False)
        return logits + self.refine(torch.cat((image, logits), dim=1)), 0.10 + 1.45 * depth.sigmoid()


class Samples(Dataset):
    def __init__(self, path):
        self.frames = []
        for summary in sorted(path.glob("*/summary.json")):
            folder = summary.parent
            if (folder / "frames.jsonl").is_file() and not (folder / "image").is_dir():
                self.frames.extend(npz_frames(folder))
            else:
                masks = sorted((folder / "mask").glob("*.png"))
                if not masks:
                    raise ValueError(f"No NPZ manifest or Gazebo masks in {folder}")
                self.frames.extend((folder, mask.name) for mask in masks)
        if not self.frames:
            raise ValueError(f"No completed map frames in {path}")

    def __len__(self):
        return len(self.frames)

    def __getitem__(self, index):
        entry = self.frames[index]
        bev_mask = np.zeros((BEV_H, BEV_W), np.uint8)
        bev_valid = np.zeros((BEV_H, BEV_W), np.uint8)
        if isinstance(entry, Path):
            with np.load(entry, allow_pickle=False) as frame:
                rgb, classes, depth = frame["rgb"], frame["classes"], frame["depth_m"]
                if "bev_mask" in frame and "bev_valid" in frame:
                    bev_mask, bev_valid = frame["bev_mask"], frame["bev_valid"]
                    if bev_mask.shape != (BEV_H, BEV_W) or bev_valid.shape != (BEV_H, BEV_W):
                        raise ValueError(f"Unexpected BEV grid in {entry}: "
                                         f"{bev_mask.shape}, {bev_valid.shape}")
        else:
            folder, name = entry
            image = cv2.imread(str(folder / "image" / name))
            mask = cv2.imread(str(folder / "mask" / name), cv2.IMREAD_GRAYSCALE)
            depth_path = folder / "depth" / name
            mm = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED) if (folder / "depth").is_dir() else None
            if image is None or mask is None or ((folder / "depth").is_dir() and mm is None):
                raise ValueError(f"Incomplete Gazebo frame {folder / name}")
            rgb, classes = cv2.cvtColor(image, cv2.COLOR_BGR2RGB), mask
            depth = mm.astype(np.float32) / 1000 if mm is not None else np.zeros_like(mask, dtype=np.float32)
        rgb = cv2.resize(rgb, SIZE, interpolation=cv2.INTER_AREA)
        line = cv2.resize(classes, SIZE, interpolation=cv2.INTER_NEAREST) == 1
        depth = cv2.resize(depth, SIZE, interpolation=cv2.INTER_NEAREST)
        depth = np.where(line & np.isfinite(depth) & (depth > 0), depth, 0)
        return rgb, 4 * np.rint(depth * 1000).astype(np.int32) + line, bev_mask, bev_valid


def unpack(batch, device):
    values = [value.to(device, non_blocking=True) for value in batch]
    rgb, packed = values[:2]
    bev_mask, bev_valid = (values[2], values[3]) if len(values) > 2 else (None, None)
    return (rgb.permute(0, 3, 1, 2).float() / 255, (packed % 4).long(),
            (packed // 4).float() / 1000, bev_mask, bev_valid)


def background_band(line, band):
    """Return background pixels within `band` pixels of a target line pixel."""
    line = line == 1
    dilated = F.max_pool2d(line.float().unsqueeze(1), 2 * band + 1, stride=1, padding=band)
    return (dilated.squeeze(1) > 0) & ~line


def hard_background_keep(loss, background, ratio):
    """Keep the hardest `ratio` share of background pixels; keep every line pixel."""
    flat_loss, flat_background = loss.flatten(), background.flatten()
    background_loss = flat_loss[flat_background]
    keep = torch.ones_like(flat_loss, dtype=torch.bool)
    count = int(background_loss.numel() * ratio)
    if count < background_loss.numel():
        threshold = torch.topk(background_loss, max(count, 1)).values[-1]
        keep &= (~flat_background) | (flat_loss >= threshold)
    return keep.view_as(loss)


def segmentation_loss(logits, line, weights, fp_weight=1., ohem_ratio=0., boundary_band=0):
    target = line.long()
    background = target != 1
    if fp_weight == 1. and ohem_ratio <= 0 and boundary_band <= 0:
        cross = F.cross_entropy(logits, target, weight=weights)
    else:
        per_pixel = F.cross_entropy(logits, target, weight=weights, reduction="none")
        scale = torch.ones_like(per_pixel)
        if boundary_band > 0:
            boost = torch.full_like(scale, fp_weight)
            scale = torch.where(background_band(target, boundary_band),
                                torch.maximum(scale, boost), scale)
        if fp_weight != 1.:
            false_positive = background & (logits.float().softmax(1)[:, 1] > .5)
            boost = torch.full_like(scale, fp_weight)
            scale = torch.where(false_positive, torch.maximum(scale, boost), scale)
        if ohem_ratio > 0:
            scale = scale * hard_background_keep(per_pixel, background, ohem_ratio)
        cross = (per_pixel * scale).sum() / (weights[target] * scale).sum().clamp_min(1e-8)
    probability = logits.float().softmax(1)[:, 1]
    dice = (2 * (probability * line).sum() + 1) / (probability.sum() + line.sum() + 1)
    return cross + .5 * (1 - dice)


def training_loss(logits, predicted, line, depth, valid, weights, depth_weight=2.,
                  bev_weight=0., bev_mask=None, bev_valid=None,
                  fp_weight=1., ohem_ratio=0., boundary_band=0.):
    """Segmentation + depth, optionally plus the differentiable BEV projection loss."""
    seg = segmentation_loss(logits, line, weights, fp_weight, ohem_ratio, boundary_band)
    predicted_depth = predicted[:, 0] if predicted.dim() == 4 else predicted
    errors = F.smooth_l1_loss(predicted_depth.float(), depth, reduction="none", beta=.04)
    loss = seg + depth_weight * (errors * valid).sum() / valid.sum().clamp_min(1)
    if bev_weight > 0 and bev_mask is not None and bev_valid is not None:
        loss = loss + bev_weight * bev_loss(project_to_bev(logits, predicted_depth),
                                            bev_mask, bev_valid)
    return loss


def evaluate(model, batches, device):
    model.eval()
    totals = torch.zeros(11, dtype=torch.float64, device=device)
    with torch.inference_mode():
        for batch in batches:
            rgb, line, depth, bev_mask, bev_valid = unpack(batch, device)
            logits, predicted = model(rgb)
            detected, actual = logits.argmax(1) == 1, line == 1
            error = (predicted[:, 0] - depth).abs()
            near = actual & (depth >= .20) & (depth <= .95)
            far = actual & (depth >= .75) & (depth <= 1.25)
            values = [(detected & actual).sum(), (detected | actual).sum(),
                      (error * near).sum(), near.sum(), (error * far).sum(), far.sum(),
                      detected.sum(), actual.sum()]
            if bev_mask is not None and bev_valid is not None:
                projected = project_to_bev(logits, predicted[:, 0])
                target = (bev_mask > 127) & (bev_valid > 0)
                guess = (projected > .5) & (bev_valid > 0)
                values += [(guess & target).sum(), guess.sum(), target.sum()]
            else:
                values += [torch.zeros((), dtype=torch.int64, device=device) for _ in range(3)]
            totals += torch.stack(values)
    (overlap, union, near_error, near_count, far_error, far_count,
     detected_count, actual_count, bev_overlap, bev_detected, bev_actual) = totals.tolist()
    return (overlap / max(union, 1), 100 * near_error / near_count if near_count else None,
            100 * far_error / far_count if far_count else None,
            overlap / max(detected_count, 1), overlap / max(actual_count, 1),
            bev_overlap / max(bev_detected, 1), bev_overlap / max(bev_actual, 1))


def warm_start(model, path):
    state = torch.load(path, map_location="cpu", weights_only=True)
    state = dict(state.get("model", state))
    if state["classes.weight"].shape[0] == 3:
        for name in ("classes.weight", "classes.bias", "refine.bias"):
            state[name] = state[name][:2]
        state["refine.weight"] = state["refine.weight"][:2, :5]
    status = model.load_state_dict(state, strict=False)
    if status.unexpected_keys or set(status.missing_keys) - {"depth.weight", "depth.bias"}:
        raise ValueError(f"Incompatible initialization: {status}")
    print(f"warm start: {path}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, action="append", help="Dataset root; repeat for training-only replay")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--init", type=Path)
    parser.add_argument("--small-model", action="store_true")
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    parser.add_argument("--train-split", choices=("train", "val"), default="train")
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--batch", type=int, default=12)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--lr", type=float, default=4e-4)
    parser.add_argument("--depth-weight", type=float, default=2.)
    parser.add_argument("--bev-weight", type=float, default=0.)
    parser.add_argument("--fp-weight", type=float, default=1.)
    parser.add_argument("--ohem-ratio", type=float, default=0.)
    parser.add_argument("--boundary-band", type=int, default=0)
    parser.add_argument("--max-train-steps", type=int)
    parser.add_argument("--max-val-steps", type=int)
    args = parser.parse_args()
    if min(args.epochs, args.batch, args.max_train_steps or 1, args.max_val_steps or 1) < 1 or (
        args.max_train_steps == 0 or args.max_val_steps == 0 or args.workers < 0
        or args.depth_weight < 0 or args.lr <= 0 or args.bev_weight < 0 or args.fp_weight < 0
        or not 0 <= args.ohem_ratio <= 1 or args.boundary_band < 0
    ):
        parser.error("Use positive epochs, batch size, learning rate and step limits, and nonnegative "
                     "workers, depth weight, bev weight, fp weight and boundary band, with ohem ratio in [0, 1]")

    torch.manual_seed(1204)
    cv2.setNumThreads(1)
    device = torch.device(args.device)
    primary, *extras = args.data or [DATASET]
    training = ConcatDataset([Samples(primary / args.train_split)] + [
        Samples(root / "train" if (root / "train").is_dir() else root) for root in extras
    ])
    validation = Samples(primary / "val")
    options = dict(batch_size=args.batch, num_workers=args.workers, pin_memory=device.type == "cuda",
                   persistent_workers=args.workers > 0)
    train = DataLoader(training, shuffle=True, **options)
    val = DataLoader(validation, **options)
    model = V4(args.small_model)
    if args.init:
        warm_start(model, args.init)
    model.to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr)
    scaler = torch.amp.GradScaler("cuda", enabled=device.type == "cuda")
    weights = torch.tensor([1., 5.], device=device)
    args.out.mkdir(parents=True, exist_ok=False)
    best = float("-inf")
    best_path = args.out / "best.pt"
    print(json.dumps(dict(device=str(device), train_frames=len(training), val_frames=len(validation))), flush=True)
    for epoch in range(1, args.epochs + 1):
        model.train()
        cumulative = torch.zeros((), device=device)
        for steps, batch in enumerate(islice(train, args.max_train_steps), 1):
            rgb, line, depth, bev_mask, bev_valid = unpack(batch, device)
            valid = (line == 1) & (depth >= .15) & (depth <= 1.4)
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast(device.type, enabled=device.type == "cuda"):
                logits, predicted = model(rgb)
                loss = training_loss(logits, predicted, line, depth, valid, weights,
                                     args.depth_weight, args.bev_weight, bev_mask, bev_valid,
                                     args.fp_weight, args.ohem_ratio, args.boundary_band)
            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()
            cumulative += loss.detach()
        (iou, near_mae, far_mae, precision, recall,
         bev_precision, bev_recall) = evaluate(model, islice(val, args.max_val_steps), device)
        row = dict(epoch=epoch, train_steps=steps, loss=float(cumulative / steps), line_iou=iou,
                   precision=precision, recall=recall,
                   bev_precision=bev_precision, bev_recall=bev_recall,
                   line_depth_mae_cm=near_mae, far_line_depth_mae_cm=far_mae)
        with (args.out / "epochs.jsonl").open("a") as log:
            print(json.dumps(row), file=log)
        print(json.dumps(row), flush=True)
        quality = iou - .01 * (near_mae or 0) if args.depth_weight else iou
        if quality > best:
            best = quality
            torch.save(model.state_dict(), best_path)
    model.cpu().eval()
    model.load_state_dict(torch.load(best_path, map_location="cpu", weights_only=True))
    output = args.out / "line_seg_depth_v4.onnx"
    torch.onnx.export(model, torch.zeros(1, 3, *SIZE[::-1]), str(output), input_names=["rgb"],
                      output_names=["logits", "depth_m"], opset_version=17, dynamo=False)
    print(f"exported {output}", flush=True)


if __name__ == "__main__":
    main()
