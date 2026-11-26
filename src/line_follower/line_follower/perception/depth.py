"""Learned line + depth perception: the line mask from the network, its position from depth.

The v4 network (b-dlseg experiments/dl/train_v4.py) takes the RGB image (320x240, /255) and
returns per-pixel line logits and metric camera-axis depth. Every line pixel then becomes a
3D point in the camera frame and is projected onto the BEV grid (x forward, y left).

Parameters: depth.model_path (ONNX), depth.min_logit_margin, depth.max_depth,
depth.splat ('dilate': grow projected points by 2 px, widens the line by ~2 cm;
'close': close gaps between projected points; 'centre': project one pixel per
horizontal mask run and close gaps). depth.use_pitch levels the projected points
with the IMU for the yaw-only map transform. Both defaults preserve legacy projection.
"""
import math
from pathlib import Path

import cv2
import numpy as np

from .. import vision
from .base import Perception, PerceptionOutput

W_IN, H_IN = 320, 240
REST_PITCH = math.radians(0.33)
CAM_MOUNT_Z = 0.093  # Camera height above the Waffle Pi model origin (SDF).


class DepthPerception(Perception):
    def __init__(self, node):
        super().__init__(node)
        node.declare_parameter('depth.model_path', '')
        node.declare_parameter('depth.min_logit_margin', 0.0)
        node.declare_parameter('depth.max_depth', 1.4)
        node.declare_parameter('depth.splat', 'dilate')
        node.declare_parameter('depth.use_pitch', False)
        path = node.get_parameter('depth.model_path').value
        if not path or not Path(path).is_file():
            raise FileNotFoundError(f"depth.model_path not found: {path!r}")
        self.margin = node.get_parameter('depth.min_logit_margin').value
        self.max_depth = node.get_parameter('depth.max_depth').value
        self.splat = node.get_parameter('depth.splat').value
        self.use_pitch = node.get_parameter('depth.use_pitch').value
        if self.splat not in ('dilate', 'close', 'centre'):
            raise ValueError(f"depth.splat must be 'dilate', 'close', or 'centre': {self.splat!r}")
        self.sess = self.net = self.trt = None
        from ..trt_runner import resolve_model
        model, is_engine = resolve_model(path)
        if is_engine:
            from ..trt_runner import TrtRunner
            self.trt = TrtRunner(model)                 # Jetson: serialized TensorRT engine
            node.get_logger().info(f'depth perception: TensorRT engine {model}')
        else:
            try:
                import onnxruntime as ort
                avail = ort.get_available_providers()
                provs = [p for p in ("TensorrtExecutionProvider", "CUDAExecutionProvider", "CPUExecutionProvider")
                         if p in avail]
                opts = ort.SessionOptions()
                opts.intra_op_num_threads = 2
                self.sess = ort.InferenceSession(path, opts, providers=provs)
            except Exception:
                cv2.setNumThreads(2)                  # one node must not take every core
                self.net = cv2.dnn.readNetFromONNX(path)
        # pixel rays of the network grid in the full camera image
        u = (np.arange(W_IN) + 0.5) * (vision.W / W_IN) - 0.5
        v = (np.arange(H_IN) + 0.5) * (vision.H / H_IN) - 0.5
        self.ru = (u - vision.CX) / vision.FX               # right per unit depth
        self.rv = (v - vision.CY) / vision.FX               # down per unit depth
        self.kernel = np.ones((3, 3), np.uint8)

    def infer(self, bgr):
        rgb = cv2.resize(bgr, (W_IN, H_IN), interpolation=cv2.INTER_AREA)[:, :, ::-1]
        x = (rgb.astype(np.float32) / 255.0).transpose(2, 0, 1)[None].copy()
        if self.trt is not None:
            out = self.trt.run({self.trt.inputs[0]: x})
            names = self.trt.output_names
            logits = out.get("logits", out[names[0]])
            depth = out.get("depth_m", out[names[-1]])
        elif self.sess is not None:
            logits, depth = self.sess.run(None, {"rgb": x})
        else:
            self.net.setInput(x)
            logits, depth = self.net.forward(["logits", "depth_m"])
        return logits[0], depth[0, 0]

    def process(self, bgr, gray, pitch, x_max):
        logits, depth = self.infer(bgr)
        line = (logits[1] - logits[0]) > self.margin
        line &= (depth > 0.1) & (depth < self.max_depth)
        if self.splat == 'centre':
            # Pad so runs touching either image edge have a start and an end.
            edges = np.diff(np.pad(line.astype(np.int8), ((0, 0), (1, 1))), axis=1)
            vv, starts = np.nonzero(edges == 1)
            _, ends = np.nonzero(edges == -1)
            uu = (starts + ends - 1) // 2
        else:
            vv, uu = np.nonzero(line)
        bev_line = np.zeros((vision.BH, vision.BW), np.uint8)
        # valid footprint as the flat projection sees it (keeps the node's range gating)
        _, valid = vision.bev_warp(gray, pitch, x_max)
        if len(vv):
            z = depth[vv, uu]
            down = self.rv[vv] * z
            if self.use_pitch:
                # The stage supplies image-time pitch, but substitutes REST_PITCH
                # on steep grades. Its process interface does not include the image
                # stamp, so use the latest raw IMU sample when that happens.
                imu = getattr(self.node, 'imu', ())
                theta = imu[-1][1] if pitch == REST_PITCH and imu else pitch
                if not math.isfinite(theta):
                    theta = REST_PITCH
                x = ((vision.CAM_X + z) * math.cos(theta)
                     - (CAM_MOUNT_Z - down) * math.sin(theta))
            else:
                x = vision.CAM_X + z * math.cos(REST_PITCH) + down * math.sin(REST_PITCH)
            y = -self.ru[uu] * z                               # left
            col = np.round((vision.Y_HALF - y) / vision.RES).astype(int)
            row = np.round((vision.X_FAR - x) / vision.RES).astype(int)
            ok = (col >= 0) & (col < vision.BW) & (row >= 0) & (row < vision.BH) & (x <= x_max)
            bev_line[row[ok], col[ok]] = 255
            if self.splat in ('close', 'centre'):
                bev_line = cv2.morphologyEx(bev_line, cv2.MORPH_CLOSE, self.kernel, iterations=2)
            else:
                bev_line = cv2.dilate(bev_line, self.kernel, iterations=2)
            bev_line &= valid
        return PerceptionOutput(line=bev_line, valid=valid, obstacle=None)
