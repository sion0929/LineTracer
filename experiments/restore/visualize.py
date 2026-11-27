#!/usr/bin/env python3
"""Worst restoration cases on the validation set: what was seen, the true route, the prediction.

For each model: the 10 samples that miss the most never-seen route, and the 10 that draw the
most line where there is none. Each row: input (seen line red, seen area dark green), true
route (white), prediction (probability), overlay (true green, predicted red, both yellow).

  python experiments/restore/visualize.py --val val.npz --model R1=r1.onnx --model R3=best.pt:24 --out dir
"""
import argparse
from pathlib import Path

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def predict(spec, X):
    path, _, width = spec.partition(":")
    if path.endswith(".onnx"):
        import onnxruntime as ort
        s = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        out = [s.run(None, {"x": X[i:i + 64].astype(np.float32)})[0] for i in range(0, len(X), 64)]
        return 1 / (1 + np.exp(-np.concatenate(out)[:, 0]))
    import torch
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from train import UNet
    w = int(width or 16)
    net = UNet((w, 2 * w, 4 * w, 6 * w, 8 * w))
    net.load_state_dict(torch.load(path, map_location="cpu"))
    net.eval()
    out = []
    with torch.no_grad():
        for i in range(0, len(X), 64):
            out.append(torch.sigmoid(net(torch.from_numpy(X[i:i + 64]).float()))[:, 0].numpy())
    return np.concatenate(out)


def per_sample(pred, Y, U):
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    missed, false = [], []
    for p, y, u in zip(pred, Y, U):
        pd = cv2.dilate(p, k)
        yd = cv2.dilate(y, k)
        missed.append(((u > 0) & (pd == 0)).sum() / max(1, u.sum()) if u.sum() >= 15 else 0.0)
        false.append(int(((p > 0) & (yd == 0)).sum()))
    return np.array(missed), np.array(false)


def panel(ax_row, x, y, prob, title):
    seen = np.zeros(x.shape[1:] + (3,))
    seen[..., 1] = x[1] * 0.3
    seen[..., 0] = x[0]
    ax_row[0].imshow(seen)
    ax_row[0].set_ylabel(title, fontsize=8)
    ax_row[1].imshow(y, cmap="gray")
    ax_row[2].imshow(prob, cmap="magma", vmin=0, vmax=1)
    ov = np.zeros(x.shape[1:] + (3,))
    pr = prob > 0.5
    ov[..., 1] = y
    ov[..., 0] = pr
    ax_row[3].imshow(ov)
    # robot at x = 0: row (1.2 - 0) / 0.0125 = 96, centre column 64
    for a in ax_row:
        a.plot(64, 96, "^", color="#2e86c1", ms=5)
        a.set_xticks([])
        a.set_yticks([])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--val", type=Path, required=True)
    ap.add_argument("--model", action="append", required=True, help="NAME=path.onnx or NAME=best.pt:width")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--n", type=int, default=10)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    z = np.load(a.val)
    X, Y, U = z["x"], z["y"], z["unseen"]
    for spec in a.model:
        name, _, path = spec.partition("=")
        prob = predict(path, X)
        pred = (prob > 0.5).astype(np.uint8)
        missed, false = per_sample(pred, Y, U)
        for kind, score in (("missed_unseen", missed), ("false_line", false)):
            idx = np.argsort(-score)[:a.n]
            fig, axs = plt.subplots(a.n, 4, figsize=(8, 2.1 * a.n))
            for r, i in enumerate(idx):
                label = f"#{i} missed {missed[i]:.0%}" if kind == "missed_unseen" else f"#{i} false {false[i]} px"
                panel(axs[r], X[i], Y[i], prob[i], label)
            for c, t in enumerate(("seen line / area (input)", "true route (GT)", "predicted route", "GT green / ours red")):
                axs[0, c].set_title(t, fontsize=9)
            fig.suptitle(f"{name}: worst {a.n} by {kind.replace('_', ' ')}", y=0.995)
            fig.tight_layout()
            f = a.out / f"{name}_{kind}.png"
            fig.savefig(f, dpi=70)
            plt.close(fig)
            print(name, kind, "mean of worst", float(score[idx].mean()), "->", f)


if __name__ == "__main__":
    main()
