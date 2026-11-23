"""Camera-only segmentation API for the experimental follower."""

from functools import lru_cache
import os
from pathlib import Path

import cv2
import numpy as np

from .geometry import HEIGHT, WIDTH

INPUT_SIZE = (320, 240)
DEFAULT_MODEL = Path(__file__).with_name("line_seg.onnx")


def prepare(bgr_img):
    if bgr_img.shape != (HEIGHT, WIDTH, 3) or bgr_img.dtype != np.uint8:
        raise ValueError(f"Expected uint8 BGR image of shape ({HEIGHT}, {WIDTH}, 3)")
    small = cv2.resize(bgr_img, INPUT_SIZE, interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray(small[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255


def decode(scores):
    labels = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    best = cv2.resize(scores[0, 0], (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)
    for category in (1, 2):
        candidate = cv2.resize(scores[0, category], (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)
        wins = candidate > best
        labels[wins] = category
        np.maximum(best, candidate, out=best)
    return labels == 1, labels == 2


class Segmenter:
    """Load an ONNX model once. CPU is the default and needs only onnxruntime."""

    def __init__(self, model_path, threads=4, device="cpu"):
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        if device not in ("cpu", "cuda"):
            raise ValueError(f"Unknown ONNX device {device!r}")
        providers = ["CPUExecutionProvider"]
        if device == "cuda":
            if "CUDAExecutionProvider" not in ort.get_available_providers():
                raise RuntimeError("CUDA inference requires onnxruntime-gpu and compatible CUDA libraries")
            providers.insert(0, "CUDAExecutionProvider")
        self.session = ort.InferenceSession(str(model_path), options, providers=providers)
        if self.session.get_providers()[0] != providers[0]:
            raise RuntimeError(f"ONNX Runtime failed to activate {providers[0]}")
        self.input_name = self.session.get_inputs()[0].name

    def __call__(self, bgr_img):
        scores, = self.session.run(None, {self.input_name: prepare(bgr_img)})
        return decode(scores)


@lru_cache(maxsize=1)
def _default_segmenter():
    path = Path(os.environ.get("LINE_SEG_ONNX", str(DEFAULT_MODEL)))
    return Segmenter(path)


def segment(bgr_img):
    """Return (line_mask, obstacle_mask), each a (480, 640) boolean array."""
    return _default_segmenter()(bgr_img)
