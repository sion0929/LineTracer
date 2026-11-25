#!/usr/bin/env python3
"""Latency of the perception models on the target (Jetson Orin Nano 8GB) or the grading PC.

Runs every ONNX in experiments/jetson/models/ with each backend that is available:
  onnxruntime TensorrtExecutionProvider (FP16), CUDAExecutionProvider, CPUExecutionProvider
  OpenCV DNN (CPU, and CUDA if OpenCV was built with it)
and prints median / p95 / max latency per frame. The budget is 33 ms per camera frame
(30 Hz) for everything; the segmentation(+depth) model runs every frame, the restoration
model only when the traced line runs out.

  python3 experiments/jetson/bench_models.py [--runs 200] [--threads 4] [--out result.json]

No ROS needed. On the Jetson (JetPack 6): pip install onnxruntime-gpu from the Jetson AI Lab
index, or use OpenCV DNN only. Copy the whole experiments/jetson folder to the board.
"""
import argparse
import json
import platform
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def input_shape(path):
    try:
        import onnx
        m = onnx.load(str(path))
        dims = [d.dim_value or 1 for d in m.graph.input[0].type.tensor_type.shape.dim]
        return dims
    except Exception:
        name = path.name
        return [1, 2, 128, 128] if "restore" in name else [1, 3, 240, 320]


def bench(fn, runs):
    for _ in range(10):
        fn()
    t = []
    for _ in range(runs):
        t0 = time.perf_counter()
        fn()
        t.append((time.perf_counter() - t0) * 1000)
    t = np.array(t)
    return dict(median_ms=round(float(np.median(t)), 2), p95_ms=round(float(np.percentile(t, 95)), 2),
                max_ms=round(float(t.max()), 2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=200)
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--out", type=Path, default=HERE / f"bench_{platform.node()}.json")
    a = ap.parse_args()
    res = {"host": platform.node(), "machine": platform.machine(), "results": []}
    models = sorted((HERE / "models").glob("*.onnx"))
    try:
        import onnxruntime as ort
        provs = ort.get_available_providers()
    except Exception:
        ort, provs = None, []
    res["onnxruntime_providers"] = provs
    for mp in models:
        shape = input_shape(mp)
        x = np.random.rand(*shape).astype(np.float32)
        if ort:
            for p in ("TensorrtExecutionProvider", "CUDAExecutionProvider", "CPUExecutionProvider"):
                if p not in provs:
                    continue
                opts = ort.SessionOptions()
                opts.intra_op_num_threads = a.threads
                po = [(p, {"trt_fp16_enable": True, "trt_engine_cache_enable": True,
                           "trt_engine_cache_path": str(HERE / "trt_cache")})] if p.startswith("Tensorrt") else [p]
                try:
                    s = ort.InferenceSession(str(mp), opts, providers=po)
                    name = s.get_inputs()[0].name
                    r = bench(lambda: s.run(None, {name: x}), a.runs)
                except Exception as e:
                    r = {"error": str(e)[:200]}
                row = dict(model=mp.name, backend=f"ort:{p}", input=shape, **r)
                res["results"].append(row)
                print(row, flush=True)
        try:
            import cv2
            cv2.setNumThreads(a.threads)
            for tgt in ("cpu", "cuda"):
                net = cv2.dnn.readNetFromONNX(str(mp))
                if tgt == "cuda":
                    if not cv2.cuda.getCudaEnabledDeviceCount():
                        continue
                    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)

                def run():
                    net.setInput(x)
                    net.forward()
                row = dict(model=mp.name, backend=f"cv2:{tgt}", input=shape, **bench(run, a.runs))
                res["results"].append(row)
                print(row, flush=True)
        except Exception as e:
            print("cv2", mp.name, e)
    a.out.write_text(json.dumps(res, indent=1))
    print("saved", a.out)


if __name__ == "__main__":
    main()
