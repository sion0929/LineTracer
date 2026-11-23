"""Train the small U-Net and export the best validation checkpoint to ONNX."""

import argparse
import json
from pathlib import Path
import random
import time

import cv2
import numpy as np
import torch
from torch.nn import functional as F
from torch.utils.data import DataLoader, Dataset

from .data import check_disjoint, read_pair, split_maps
from .infer import INPUT_SIZE
from .geometry import HEIGHT, WIDTH
from .model import SmallUNet


class FrameDataset(Dataset):
    def __init__(self, maps, input_size):
        self.samples = [(directory, row["file"]) for directory, _, rows in maps for row in rows]
        self.images = np.empty((len(self.samples), input_size[1], input_size[0], 3), np.uint8)
        self.labels = np.empty(self.images.shape[:-1], np.uint8)
        for i, (directory, name) in enumerate(self.samples):
            bgr, label = read_pair(directory, name)
            self.images[i] = cv2.resize(bgr, input_size, interpolation=cv2.INTER_AREA)[:, :, ::-1]
            self.labels[i] = cv2.resize(label, input_size, interpolation=cv2.INTER_NEAREST_EXACT)

    def __len__(self):
        return len(self.images)

    def __getitem__(self, index):
        return self.images[index], self.labels[index]


def apply_shadow(images, center, slope, softness, strength, opposite_edge=None):
    """Attenuate RGB on one side of a softly edged screen-space line."""
    height, width = images.shape[-2:]
    x = torch.linspace(-1, 1, width, device=images.device)[None, None, :]
    y = torch.linspace(-1, 1, height, device=images.device)[None, :, None]
    distance = y - center[:, None, None] - slope[:, None, None] * x
    alpha = (distance / softness[:, None, None] + 0.5).clamp(0, 1)
    if opposite_edge is not None:
        edge_center, edge_slope = opposite_edge
        other_distance = edge_center[:, None, None] + edge_slope[:, None, None] * x - y
        alpha *= (other_distance / softness[:, None, None] + 0.5).clamp(0, 1)
    scale = 1 - (1 - strength[:, None, None]) * alpha
    return images * scale[:, None]


def augment_batch(images, labels, shadow_prob=0.0, official_style=False):
    batch = len(images)
    device = images.device
    shape = (batch, 1, 1, 1)

    def rand(dims):
        return torch.rand(dims, device=device)

    flip = rand(shape) < 0.5
    images = torch.where(flip, images.flip((-1,)), images)
    labels = torch.where(flip[:, 0], labels.flip((-1,)), labels)
    brightness = 0.65 + 0.65 * rand(shape)
    balance = 0.85 + 0.30 * rand((batch, 3, 1, 1))
    gamma = 0.75 + 0.50 * rand(shape)
    bias = 0.16 * rand(shape) - 0.08
    images = (images.pow(gamma) * brightness * balance + bias).clamp_(0, 1)
    if official_style:
        x = torch.linspace(-1, 1, images.shape[-1], device=device)[None, None, None, :]
        y = torch.linspace(-1, 1, images.shape[-2], device=device)[None, None, :, None]
        gain = 1 + (0.24 * rand(shape) - 0.12) * x + (0.22 * rand(shape) - 0.11) * y
        images = (images * gain).clamp_(0, 1)
    if shadow_prob:
        selected = rand(batch) < shadow_prob
        strength = torch.where(selected, 0.30 + 0.45 * rand(batch), 1.0)
        opposite_edge = None
        if official_style:
            opposite_edge = (0.35 + 1.10 * rand(batch), 1.3 * (rand(batch) - 0.5))
        images = apply_shadow(
            images, -0.1 + 0.9 * rand(batch), 0.9 * (rand(batch) - 0.5),
            0.015 + 0.12 * rand(batch), strength, opposite_edge,
        )
    return images, labels


def soft_dice(logits, target):
    probability = logits.softmax(dim=1)[:, 1:]
    foreground = torch.stack((target == 1, target == 2), dim=1)
    total = (0, 2, 3)
    overlap = (probability * foreground).sum(total)
    union = probability.sum(total) + foreground.sum(total)
    return 1 - ((2 * overlap + 1) / (union + 1)).mean()


def confusion(prediction, target):
    return torch.bincount((target * 3 + prediction).flatten(), minlength=9).reshape(3, 3)


def iou_from_confusion(confusion_matrix):
    true_positive = np.diag(confusion_matrix)
    union = confusion_matrix.sum(axis=0) + confusion_matrix.sum(axis=1) - true_positive
    return [float(tp / u) if u else None for tp, u in zip(true_positive, union)]


@torch.inference_mode()
def validate(model, batches, device):
    counts = torch.zeros((3, 3), dtype=torch.long, device=device)
    model.eval()
    for rgb, target in batches:
        inputs = rgb.to(device, non_blocking=True).permute(0, 3, 1, 2).float().div_(255)
        target = target.to(device, non_blocking=True).long()
        logits = F.interpolate(model(inputs), target.shape[-2:], mode="bilinear", align_corners=False)
        counts += confusion(logits.argmax(dim=1), target)
    return iou_from_confusion(counts.cpu().numpy())


def export_onnx(model, path, input_size=INPUT_SIZE):
    import onnx

    model = model.cpu().eval()
    dummy = torch.zeros(1, 3, input_size[1], input_size[0])
    torch.onnx.export(model, dummy, str(path), input_names=["rgb"], output_names=["logits"],
                      opset_version=17, dynamo=False)
    onnx.checker.check_model(str(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=24)
    parser.add_argument("--batch-size", type=int, default=24)
    parser.add_argument("--seed", type=int, default=12)
    parser.add_argument("--device", choices=["cpu", "cuda"], default="cuda")
    parser.add_argument("--shadow-prob", type=float, default=0.0)
    parser.add_argument("--official-style", action="store_true")
    parser.add_argument("--full-resolution", action="store_true")
    parser.add_argument("--initialize-from", type=Path, help="Initialize compatible weights from a checkpoint")
    parser.add_argument("--learning-rate", type=float, default=0.002)
    args = parser.parse_args()
    if min(args.epochs, args.batch_size) < 1:
        parser.error("epochs and batch size must be positive")
    if not 0 <= args.shadow_prob <= 1:
        parser.error("shadow probability must be between 0 and 1")
    if not args.learning_rate > 0:
        parser.error("learning rate must be positive")
    args.out.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    torch.set_num_threads(4)
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    device = torch.device(args.device)
    use_cuda = device.type == "cuda"
    if use_cuda:
        torch.backends.cudnn.benchmark = True
        torch.backends.cudnn.allow_tf32 = True
    training = split_maps(args.data, "train")
    validation = split_maps(args.data, "val")
    check_disjoint(training, validation)
    if any(summary["map_seed"] is None for maps in (training, validation) for _, summary, _ in maps):
        raise ValueError("Official track is reserved for testing")
    input_size = (WIDTH, HEIGHT) if args.full_resolution else INPUT_SIZE
    training_frames = FrameDataset(training, input_size)
    validation_frames = FrameDataset(validation, input_size)
    train_batches = DataLoader(training_frames, batch_size=args.batch_size, shuffle=True, pin_memory=use_cuda)
    val_batches = DataLoader(validation_frames, batch_size=args.batch_size, pin_memory=use_cuda)
    model = SmallUNet(refine=args.full_resolution).to(device, memory_format=torch.channels_last)
    if args.initialize_from:
        state = torch.load(args.initialize_from, map_location=device, weights_only=True)
        missing, unexpected = model.load_state_dict(state, strict=False)
        if unexpected or set(missing) - {"refine.weight", "refine.bias"}:
            raise ValueError(f"Incompatible initialization: missing {missing}, unexpected {unexpected}")
    weights = torch.tensor([1.0, 5.0, 4.0], device=device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.learning_rate, weight_decay=0.001)
    schedule = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, args.epochs, eta_min=args.learning_rate / 20)
    scaler = torch.amp.GradScaler(device.type, enabled=use_cuda)
    best, best_epoch = -1.0, None
    start = time.monotonic()
    with (args.out / "epochs.jsonl").open("w") as log:
        for epoch in range(1, args.epochs + 1):
            model.train()
            losses = torch.zeros((), device=device)
            for rgb, target in train_batches:
                inputs = rgb.to(device, non_blocking=True).permute(0, 3, 1, 2).float().div_(255)
                target = target.to(device, non_blocking=True).long()
                inputs, target = augment_batch(inputs, target, args.shadow_prob, args.official_style)
                optimizer.zero_grad(set_to_none=True)
                with torch.autocast(device.type, enabled=use_cuda):
                    logits = F.interpolate(model(inputs), target.shape[-2:], mode="bilinear", align_corners=False)
                    loss = F.cross_entropy(logits, target, weight=weights) + 0.5 * soft_dice(logits.float(), target)
                scaler.scale(loss).backward()
                scaler.step(optimizer)
                scaler.update()
                losses += loss.detach()
            schedule.step()
            iou = validate(model, val_batches, device)
            score = float(np.mean([value for value in iou[1:] if value is not None]))
            row = {
                "epoch": epoch, "loss": losses.item() / len(train_batches),
                "val_iou": iou, "foreground_mean_iou": score,
            }
            print(json.dumps(row), flush=True)
            log.write(json.dumps(row) + "\n")
            log.flush()
            if score > best:
                best, best_epoch = score, epoch
                torch.save(model.state_dict(), args.out / "best.pt")
    model.load_state_dict(torch.load(args.out / "best.pt", map_location=device, weights_only=True))
    export_onnx(model, args.out / "line_seg.onnx", input_size)
    summary = {
        "train_frames": len(training_frames), "validation_frames": len(validation_frames),
        "train_maps": [summary["map_seed"] for _, summary, _ in training],
        "validation_maps": [summary["map_seed"] for _, summary, _ in validation],
        "parameters": sum(t.numel() for t in model.parameters()),
        "best_epoch": best_epoch, "val_foreground_mean_iou": best,
        "elapsed_s": time.monotonic() - start, "seed": args.seed,
        "shadow_probability": args.shadow_prob,
        "official_style": args.official_style,
        "input_size_wh": input_size,
        "initial_checkpoint": str(args.initialize_from) if args.initialize_from else None,
        "learning_rate": args.learning_rate,
    }
    (args.out / "training.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
