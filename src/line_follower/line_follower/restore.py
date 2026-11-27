"""Line restoration: continue the route where the camera has not seen the line (learned).

A small U-Net (experiments/restore/train.py) maps the robot's local line map to the route:
  input  ch0  line cells of the map, ch1  cells the camera has seen
  output route probability, 128x128, x forward -0.4..1.2 m, y -0.8..0.8 m, 1.25 cm cells
It fills gaps, the blind zone under the bumper and corners that leave the camera view.
The node asks it only when the traced path runs out ("open" and short).
"""
import math
from pathlib import Path

import cv2
import numpy as np

from .line_map import C0, RES

PRES = 0.0125
N = 128
X1, Y0 = 1.2, 0.8


class Restorer:
    def __init__(self, onnx_path):
        # onnxruntime first (TensorRT FP16 on the Jetson, CUDA, else CPU); OpenCV DNN fallback
        self.sess, self.net, self.backend = None, None, "cv2"
        try:
            import onnxruntime as ort
            avail = ort.get_available_providers()
            provs = []
            if "TensorrtExecutionProvider" in avail:
                cache = str(Path(onnx_path).parent / "trt_cache")
                provs.append(("TensorrtExecutionProvider", {"trt_fp16_enable": True,
                                                            "trt_engine_cache_enable": True,
                                                            "trt_engine_cache_path": cache}))
            provs += [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in avail]
            opts = ort.SessionOptions()
            opts.intra_op_num_threads = 2
            self.sess = ort.InferenceSession(str(onnx_path), opts, providers=provs)
            self.inp = self.sess.get_inputs()[0].name
            self.backend = self.sess.get_providers()[0]
        except Exception:
            self.net = cv2.dnn.readNetFromONNX(str(onnx_path))
        self.calls = 0

    def patch(self, lm, pose):
        """Robot-centric patch of the map (line, seen) as the network expects it."""
        px, py, th = pose
        c, s = math.cos(th), math.sin(th)
        # patch pixel (col, row) -> robot (x, y) -> odom -> map cell (j, i)
        # x = X1 - row*PRES, y = Y0 - col*PRES
        # X = px + x c - y s ; Y = py + x s + y c ; j = X/RES + C0 ; i = Y/RES + C0
        k = PRES / RES
        a = np.float32([
            [s * k, -c * k, (px + X1 * c - Y0 * s) / RES + C0],
            [-c * k, -s * k, (py + X1 * s + Y0 * c) / RES + C0],
        ])
        line = cv2.warpAffine(lm.grid, a, (N, N), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
        seen = cv2.warpAffine(lm.seen, a, (N, N), flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP)
        return (line > 60).astype(np.float32), (seen > 0).astype(np.float32)

    def route(self, lm, pose):
        line, seen = self.patch(lm, pose)
        blob = np.stack([line, seen])[None].astype(np.float32)
        if self.sess is not None:
            logit = self.sess.run(None, {self.inp: blob})[0][0, 0]
        else:
            self.net.setInput(blob)
            logit = self.net.forward()[0, 0]
        self.calls += 1
        return 1.0 / (1.0 + np.exp(-logit))

    @staticmethod
    def to_robot(col, row):
        return X1 - (np.asarray(row) + 0.5) * PRES, Y0 - (np.asarray(col) + 0.5) * PRES

    def extend_ridge(self, prob, start, known, length=0.4, thresh=0.6, *, min_mean=0.0):
        """Longest walk from `start` through the confident part of the prediction (any angle:
        acute corners and U-turns too), avoiding the already known path `known` (robot frame).
        Returns robot-frame points ~2 cm apart, at most `length` long.
        Rejects the chain if its sampled mean probability is below `min_mean`."""
        from collections import deque
        m = prob > thresh
        if not m.any():
            return []
        def px(q):
            return int(round((Y0 - q[1]) / PRES - 0.5)), int(round((X1 - q[0]) / PRES - 0.5))
        c0, r0 = px(start)
        # seed: nearest confident pixel within 4 px of the start
        best = None
        for dr in range(-4, 5):
            for dc in range(-4, 5):
                r, c = r0 + dr, c0 + dc
                if 0 <= r < N and 0 <= c < N and m[r, c]:
                    d = dr * dr + dc * dc
                    if best is None or d < best[0]:
                        best = (d, r, c)
        if best is None:
            return []
        block = np.zeros_like(m)
        for q in known[:-8]:                    # do not walk back along the known line (keep the last ~16 cm open)
            c, r = px(q)
            if 0 <= r < N and 0 <= c < N:
                block[max(0, r - 2):r + 3, max(0, c - 2):c + 3] = True
        m = m & ~block
        _, rs, cs = best
        m[rs, cs] = True
        dist = np.full(m.shape, -1, np.int32)
        par = {}
        dist[rs, cs] = 0
        dq = deque([(rs, cs)])
        far = (rs, cs)
        while dq:
            r, c = dq.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < N and 0 <= cc < N and m[rr, cc] and dist[rr, cc] < 0:
                    dist[rr, cc] = dist[r, c] + 1
                    par[(rr, cc)] = (r, c)
                    dq.append((rr, cc))
                    if dist[rr, cc] > dist[far]:
                        far = (rr, cc)
        chain = [far]
        while chain[-1] != (rs, cs):
            chain.append(par[chain[-1]])
        chain.reverse()
        out, acc, last = [], 0.0, start
        for r, c in chain[1::2]:                 # every 2nd pixel ~ 2.5 cm
            x, y = X1 - (r + 0.5) * PRES, Y0 - (c + 0.5) * PRES
            acc += math.hypot(x - last[0], y - last[1])
            if acc > length:
                break
            out.append((x, y))
            last = (x, y)
        if out and min_mean > 0:
            cols, rows = zip(*(px(q) for q in out))
            if float(prob[rows, cols].mean()) < min_mean:
                return []
        return out

    def extend(self, prob, start, heading, length=0.6, step=0.02, thresh=0.5):
        """Follow the predicted route from start (robot frame) for up to `length`."""
        x, y, h = start[0], start[1], heading
        out = []
        for _ in range(int(length / step)):
            best = None
            for da in np.deg2rad(np.arange(-60, 61, 6)):
                nx, ny = x + step * math.cos(h + da), y + step * math.sin(h + da)
                col, row = (Y0 - ny) / PRES, (X1 - nx) / PRES
                if not (0 <= col < N - 1 and 0 <= row < N - 1):
                    continue
                p = prob[int(row), int(col)] - 0.002 * abs(math.degrees(da))   # prefer straight
                if best is None or p > best[0]:
                    best = (p, nx, ny, h + da)
            if best is None or best[0] < thresh:
                break
            _, x, y, h = best
            out.append((x, y))
        return out


def load(path):
    p = Path(path)
    return Restorer(p) if p.is_file() else None
