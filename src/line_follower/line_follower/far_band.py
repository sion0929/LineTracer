"""Learned far-band line detector (dual resolution, see vision.far_band_mask).

A small U-Net (experiments/farband/train.py) segments the line in a 64x640 strip of the
FULL-resolution camera image just below the horizon (~0.8-1.9 m ahead on flat ground), where a
line crossing the view is only ~1 px thick. The strip follows the camera pitch.
"""
import math
from pathlib import Path

import cv2
import numpy as np

from . import vision

ROW0, ROWS = 256, 64


class FarBandModel:
    def __init__(self, onnx_path):
        self.sess, self.net = None, None
        try:
            import onnxruntime as ort
            avail = ort.get_available_providers()
            provs = []
            if "TensorrtExecutionProvider" in avail:
                provs.append(("TensorrtExecutionProvider", {"trt_fp16_enable": True, "trt_engine_cache_enable": True,
                                                            "trt_engine_cache_path": str(Path(onnx_path).parent / "trt_cache")}))
            provs += [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in avail]
            opts = ort.SessionOptions()
            opts.intra_op_num_threads = 2
            self.sess = ort.InferenceSession(str(onnx_path), opts, providers=provs)
            self.inp = self.sess.get_inputs()[0].name
        except Exception:
            self.net = cv2.dnn.readNetFromONNX(str(onnx_path))
        side = Path(onnx_path).with_suffix(".json")
        import json
        self.in_w = json.loads(side.read_text()).get("width", vision.W) if side.is_file() else vision.W

    def mask(self, bgr, theta):
        """Full-size (H, W) uint8 0/255 image with the predicted line inside the strip."""
        r0 = int(round(ROW0 + vision.FX * math.tan(theta)))
        r0 = max(0, min(vision.H - ROWS, r0))
        strip = bgr[r0:r0 + ROWS, :, ::-1]
        if self.in_w != vision.W:                       # columns downscaled, rows stay full resolution
            strip = cv2.resize(strip, (self.in_w, ROWS), interpolation=cv2.INTER_AREA)
        strip = strip.astype(np.float32) / 255.0      # RGB
        blob = np.ascontiguousarray(strip.transpose(2, 0, 1)[None])
        if self.sess is not None:
            logit = self.sess.run(None, {self.inp: blob})[0][0, 0]
        else:
            self.net.setInput(blob)
            logit = self.net.forward()[0, 0]
        out = np.zeros((vision.H, vision.W), np.uint8)
        m = (logit > 0).astype(np.uint8) * 255
        if m.shape[1] != vision.W:
            m = cv2.resize(m, (vision.W, ROWS), interpolation=cv2.INTER_NEAREST)
        out[r0:r0 + ROWS] = m
        return out


def load(path):
    p = Path(path)
    return FarBandModel(p) if p.is_file() else None
