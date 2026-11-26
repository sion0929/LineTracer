#!/usr/bin/env python3
"""Train the line-only V4 segmentation and metric-depth model."""
import argparse
import json
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


def block(inputs, outputs, stride=1):
    return nn.Sequential(
        nn.Conv2d(inputs, outputs, 3, stride=stride, padding=1, bias=False),
        nn.BatchNorm2d(outputs),
        nn.ReLU(inplace=True),
    )


def upsample(feature, skip):
    return torch.cat((F.interpolate(feature, size=skip.shape[-2:], mode="nearest"), skip), dim=1)


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
        if isinstance(entry, Path):
            with np.load(entry, allow_pickle=False) as frame:
                rgb, classes, depth = frame["rgb"], frame["classes"], frame["depth_m"]
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
        return rgb, 4 * np.rint(depth * 1000).astype(np.int32) + line


def unpack(batch, device):
    rgb, packed = (value.to(device, non_blocking=True) for value in batch)
    return rgb.permute(0, 3, 1, 2).float() / 255, (packed % 4).long(), (packed // 4).float() / 1000


def evaluate(model, batches, device):
    model.eval()
    totals = torch.zeros(6, dtype=torch.float64, device=device)
    with torch.inference_mode():
        for batch in batches:
            rgb, line, depth = unpack(batch, device)
            logits, predicted = model(rgb)
            detected, actual = logits.argmax(1) == 1, line == 1
            error = (predicted[:, 0] - depth).abs()
            near = actual & (depth >= .20) & (depth <= .95)
            far = actual & (depth >= .75) & (depth <= 1.25)
            totals += torch.stack(((detected & actual).sum(), (detected | actual).sum(),
                                   (error * near).sum(), near.sum(), (error * far).sum(), far.sum()))
    overlap, union, near_error, near_count, far_error, far_count = totals.tolist()
    return (overlap / max(union, 1), 100 * near_error / near_count if near_count else None,
            100 * far_error / far_count if far_count else None)


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
    parser.add_argument("--max-train-steps", type=int)
    parser.add_argument("--max-val-steps", type=int)
    args = parser.parse_args()
    if min(args.epochs, args.batch, args.max_train_steps or 1, args.max_val_steps or 1) < 1 or (
        args.max_train_steps == 0 or args.max_val_steps == 0 or args.workers < 0
        or args.depth_weight < 0 or args.lr <= 0
    ):
        parser.error("Use positive epochs, batch size, learning rate and step limits, and nonnegative workers and depth weight")

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
            rgb, line, depth = unpack(batch, device)
            valid = (line == 1) & (depth >= .15) & (depth <= 1.4)
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast(device.type, enabled=device.type == "cuda"):
                logits, predicted = model(rgb)
                probability = logits.float().softmax(1)[:, 1]
                dice = (2 * (probability * line).sum() + 1) / (probability.sum() + line.sum() + 1)
                seg = F.cross_entropy(logits, line, weight=weights) + .5 * (1 - dice)
                errors = F.smooth_l1_loss(predicted[:, 0].float(), depth, reduction="none", beta=.04)
                loss = seg + args.depth_weight * (errors * valid).sum() / valid.sum().clamp_min(1)
            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()
            cumulative += loss.detach()
        iou, near_mae, far_mae = evaluate(model, islice(val, args.max_val_steps), device)
        row = dict(epoch=epoch, train_steps=steps, loss=float(cumulative / steps), line_iou=iou,
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
