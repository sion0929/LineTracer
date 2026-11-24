"""Deep-learning perception (owner: deep-learning teammate).

Contract: same BEV grid as vision.line_mask. A ready path:
  1. run a segmentation model on the camera image -> per-pixel line / obstacle masks
     (experiments/dl/infer.py has segment(bgr) -> (line_mask, obstacle_mask) 480x640),
  2. warp them to BEV with vision.bev_warp(mask, pitch, x_max) (same homography the
     rule plugin uses), 3. return PerceptionOutput(line, valid, obstacle).

Until a model is configured this plugin refuses to start, so a run never silently
falls back to the rule plugin.
"""
import os

import numpy as np

from .. import vision
from .base import Perception, PerceptionOutput


class DlPerception(Perception):
    def __init__(self, node):
        super().__init__(node)
        node.declare_parameter('dl.model_path', '')
        node.declare_parameter('dl.device', 'cpu')
        path = node.get_parameter('dl.model_path').value
        if not path or not os.path.isfile(path):
            raise RuntimeError("perception 'dl' needs parameter dl.model_path (ONNX file)")
        self.model_path = path
        self.device = node.get_parameter('dl.device').value
        self.session = self.load(path, self.device)

    def load(self, path, device):
        """TODO(dl teammate): load the model (e.g. onnxruntime.InferenceSession)."""
        raise NotImplementedError('DlPerception.load: plug in the segmentation model here')

    def segment(self, bgr):
        """TODO(dl teammate): return (line_mask, obstacle_mask), each HxW bool."""
        raise NotImplementedError('DlPerception.segment: run the model here')

    def process(self, bgr, gray, pitch, x_max):
        line_px, obst_px = self.segment(bgr)
        line, valid = vision.bev_warp(line_px.astype(np.uint8) * 255, pitch, x_max)
        obstacle = None
        if obst_px is not None:
            obstacle, _ = vision.bev_warp(obst_px.astype(np.uint8) * 255, pitch, x_max)
        return PerceptionOutput(line=line, valid=valid, obstacle=obstacle)
