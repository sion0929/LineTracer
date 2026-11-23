"""Compare geometric-mask IoU and measure end-to-end segmentation latency."""

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from .data import check_disjoint, read_pair, split_maps
from .infer import Segmenter, decode, prepare


def confusion(target, prediction):
    indices = target * 3 + prediction
    return np.bincount(indices.ravel(), minlength=9).reshape(3, 3)


def class_iou(matrix):
    hits = matrix.diagonal()
    union = matrix.sum(0) + matrix.sum(1) - hits
    return {label: float(hit / count) if count else None
            for label, hit, count in zip(("background", "line", "obstacle"), hits, union)}


def prediction_mask(masks):
    line, obstacle = masks
    return line.astype(np.uint8) + 2 * obstacle.astype(np.uint8)


def show_mask(image, label):
    colors = image.copy()
    colors[label == 1] = (55, 234, 24)
    colors[label == 2] = (12, 125, 255)
    return cv2.addWeighted(image, 0.52, colors, 0.48, 0)


def failure_example(output, tag, case):
    _, name, image, target, prediction, baseline = case
    row = np.hstack([
        cv2.resize(panel, (320, 240), interpolation=cv2.INTER_AREA)
        for panel in (image, show_mask(image, target), show_mask(image, prediction), show_mask(image, baseline))
    ])
    canvas = cv2.copyMakeBorder(row, 30, 26, 0, 0, cv2.BORDER_CONSTANT, value=(25, 25, 25))
    for x, title in enumerate(("Camera", "Geometric truth", "U-Net", "OpenCV gray < 80")):
        cv2.putText(canvas, title, (x * 320 + 8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (230, 230, 230), 1)
    cv2.putText(canvas, name, (8, 288), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (230, 230, 230), 1)
    if not cv2.imwrite(str(output / f"failure-{tag}.jpg"), canvas):
        raise OSError(f"Could not write failure example {tag}")


def evaluate(segment, maps, output):
    results = {}
    frames_for_benchmark = []
    for directory, summary, records in maps:
        counts = np.zeros((2, 3, 3), dtype=np.int64)
        failures = {}
        clean_frames = 0
        false_obstacle_frames = 0
        max_false_obstacle_pixels = 0
        for index, row in enumerate(records):
            filename = row["file"]
            image, target = read_pair(directory, filename)
            if index % max(len(records) // 8, 1) == 0:
                frames_for_benchmark.append(image)
            baseline = (cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) < 80).astype(np.uint8)
            prediction = prediction_mask(segment(image))
            raw = confusion(target, prediction)
            counts[0] += raw
            counts[1] += confusion(target, baseline)
            if raw[2].sum() == 0:
                clean_frames += 1
                false_pixels = int(raw[:, 2].sum())
                false_obstacle_frames += false_pixels > 0
                max_false_obstacle_pixels = max(max_false_obstacle_pixels, false_pixels)
            scores = class_iou(raw)
            for category, number, minimum in (("line", 1, 300), ("obstacle", 2, 600)):
                if raw[number].sum() < minimum:
                    continue
                score = scores[category]
                if category not in failures or score < failures[category][0]:
                    name = f"{summary['map']} / {filename} / {category} IoU {score:.3f}"
                    failures[category] = (score, name, image, target, prediction, baseline)
        obstacle_audit = {
            "obstacle_free_frames": clean_frames,
            "false_obstacle_frames": false_obstacle_frames,
            "max_false_obstacle_pixels": max_false_obstacle_pixels,
        }
        results[summary["map"]] = (len(records), counts, obstacle_audit)
        for category, case in failures.items():
            failure_example(output, f"{summary['map']}-{category}", case)
    return results, frames_for_benchmark


def benchmark(function, frames, repeats):
    for i in range(40):
        function(frames[i % len(frames)])
    duration = []
    for i in range(repeats):
        start = time.perf_counter()
        function(frames[i % len(frames)])
        duration.append(time.perf_counter() - start)
    return {
        "samples": repeats, "mean_ms": float(np.mean(duration) * 1000),
        "median_ms": float(np.median(duration) * 1000),
        "p95_ms": float(np.percentile(duration, 95) * 1000),
        "fps": float(1 / np.mean(duration)),
    }


def torch_cuda_segment(checkpoint):
    import torch

    from .model import SmallUNet

    torch.set_num_threads(4)
    torch.backends.cudnn.allow_tf32 = True
    model = SmallUNet().cuda().eval().to(memory_format=torch.channels_last)
    model.load_state_dict(torch.load(checkpoint, map_location="cuda", weights_only=True))

    @torch.inference_mode()
    def segment(image):
        inputs = torch.from_numpy(prepare(image)).to("cuda", memory_format=torch.channels_last)
        return decode(model(inputs).cpu().numpy())

    return segment, torch.cuda.get_device_name()


def result(frames, counts):
    return {
        "frames": frames,
        "small_unet_iou": class_iou(counts[0]), "opencv_iou": class_iou(counts[1]),
        "small_unet_confusion": counts[0].tolist(), "opencv_confusion": counts[1].tolist(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--onnx", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, help="Optional checkpoint for the GPU latency measurement")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--benchmark-frames", type=int, default=240)
    args = parser.parse_args()
    if min(args.threads, args.benchmark_frames) < 1:
        parser.error("Thread count and benchmark frame count must be positive")
    args.out.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    train_maps, val_maps, test_maps = (split_maps(args.data, split) for split in ("train", "val", "test"))
    check_disjoint(train_maps, val_maps, test_maps)
    official = {summary["map"] for _, summary, _ in test_maps if summary["map_seed"] is None}
    if not official or len(official) == len(test_maps):
        raise ValueError("Test split must contain an official map and a held-out random map")
    segment = Segmenter(args.onnx, threads=args.threads)
    results, frames = evaluate(segment, test_maps, args.out)
    random = [(n, counts) for name, (n, counts, _) in results.items() if name not in official]
    rows = {
        name: {**result(n, counts), "obstacle_audit": audit}
        for name, (n, counts, audit) in results.items()
    }
    rows["heldout_random_pooled"] = result(sum(n for n, _ in random), sum(c for _, c in random))
    speed = {"onnx_cpu": {"threads": args.threads, **benchmark(segment, frames, args.benchmark_frames)}}
    if args.checkpoint:
        gpu, device = torch_cuda_segment(args.checkpoint)
        speed["pytorch_cuda"] = {"device": device, **benchmark(gpu, frames, args.benchmark_frames)}
    report = {"test": rows, "latency": speed}
    (args.out / "evaluation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
