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
from .model import SmallUNet


class FrameDataset(Dataset):
    def __init__(self, maps):
        self.samples = [(directory, row["file"]) for directory, _, rows in maps for row in rows]
        self.images = np.empty((len(self.samples), INPUT_SIZE[1], INPUT_SIZE[0], 3), np.uint8)
        self.labels = np.empty(self.images.shape[:-1], np.uint8)
        for i, (directory, name) in enumerate(self.samples):
            bgr, label = read_pair(directory, name)
            self.images[i] = cv2.resize(bgr, INPUT_SIZE, interpolation=cv2.INTER_AREA)[:, :, ::-1]
            self.labels[i] = cv2.resize(label, INPUT_SIZE, interpolation=cv2.INTER_NEAREST_EXACT)

    def __len__(self):
        return len(self.images)

    def __getitem__(self, index):
        return self.images[index], self.labels[index]


def augment_batch(images, labels):
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


def export_onnx(model, path):
    import onnx

    model = model.cpu().eval()
    dummy = torch.zeros(1, 3, INPUT_SIZE[1], INPUT_SIZE[0])
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
    args = parser.parse_args()
    if min(args.epochs, args.batch_size) < 1:
        parser.error("epochs and batch size must be positive")
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
    training_frames, validation_frames = FrameDataset(training), FrameDataset(validation)
    train_batches = DataLoader(training_frames, batch_size=args.batch_size, shuffle=True, pin_memory=use_cuda)
    val_batches = DataLoader(validation_frames, batch_size=args.batch_size, pin_memory=use_cuda)
    model = SmallUNet().to(device, memory_format=torch.channels_last)
    weights = torch.tensor([1.0, 5.0, 4.0], device=device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=0.002, weight_decay=0.001)
    schedule = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, args.epochs, eta_min=0.0001)
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
                inputs, target = augment_batch(inputs, target)
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
    export_onnx(model, args.out / "line_seg.onnx")
    summary = {
        "train_frames": len(training_frames), "validation_frames": len(validation_frames),
        "train_maps": [summary["map_seed"] for _, summary, _ in training],
        "validation_maps": [summary["map_seed"] for _, summary, _ in validation],
        "parameters": sum(t.numel() for t in model.parameters()),
        "best_epoch": best_epoch, "val_foreground_mean_iou": best,
        "elapsed_s": time.monotonic() - start, "seed": args.seed,
    }
    (args.out / "training.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
