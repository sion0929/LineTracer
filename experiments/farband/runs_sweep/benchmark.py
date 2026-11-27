"""Measure median OpenCV DNN forward latency for exported far-band models."""

import json
import statistics
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def benchmark(model):
    spec = json.loads(model.with_suffix(".json").read_text())
    shape = (1, 3, spec["rows"], spec["width"])
    blob = np.random.default_rng(0).random(shape, dtype=np.float32)
    net = cv2.dnn.readNetFromONNX(str(model))
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    net.setInput(blob, "rgb")
    for _ in range(20):
        logits = net.forward("logits")
    if logits.shape != (1, 1, *shape[-2:]) or not np.isfinite(logits).all():
        raise ValueError(f"Unexpected logits from {model}: {logits.shape}")
    elapsed_ms = []
    for _ in range(100):
        start = time.perf_counter_ns()
        net.setInput(blob, "rgb")
        net.forward("logits")
        elapsed_ms.append((time.perf_counter_ns() - start) / 1e6)
    return statistics.median(elapsed_ms)


if __name__ == "__main__":
    cv2.setNumThreads(2)
    print(f"OpenCV {cv2.__version__}, {cv2.getNumThreads()} threads")
    for filename in sys.argv[1:]:
        model = Path(filename)
        print(f"{model}: {benchmark(model):.3f} ms", flush=True)
