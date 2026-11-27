import os, sys
os.environ["LF_X_FAR"] = "1.2"
from pathlib import Path
import cv2, numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/line_follower"))
from line_follower import vision
D = ROOT / "experiments/results/lead/res_compare"
rows = []
for n in ("tile_034_ex2", "tile_009_ex1", "tile_009_ex3"):
    g = cv2.cvtColor(cv2.imread(str(D / f"{n}.png")), cv2.COLOR_BGR2GRAY)
    a, va = vision.line_mask(g, 0.0, 1.2, scale=0.5)          # all 320
    b, vb = vision.line_mask(g, 0.0, 1.2)                      # all 640
    n1, nv = vision.line_mask(g, 0.0, 0.85, scale=0.5)         # dual: near 320 ...
    f1, fv = vision.far_band_mask(g, 0.0, 0.85, 1.2)           # ... + far 640 band
    d = np.maximum(n1, f1); dv = np.maximum(nv, fv)
    tiles = []
    for (m, v), lab in (((a, va), "320 all"), ((b, vb), "640 all"), ((d, dv), "dual 320+640 band")):
        t = cv2.cvtColor(np.where(v > 0, m, 60).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        r = int((vision.X_FAR - 0.85) / vision.RES); cv2.line(t, (0, r), (t.shape[1], r), (0, 140, 255), 1)
        cv2.putText(t, lab, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1)
        tiles.append(t)
    rows.append(np.hstack(tiles))
cv2.imwrite(str(D / "far_band_check.png"), cv2.resize(np.vstack(rows), None, fx=1.5, fy=1.5, interpolation=cv2.INTER_NEAREST))
import time
t = time.perf_counter()
for _ in range(200): vision.line_mask(g, 0.0, 0.85, scale=0.5)
t1 = time.perf_counter()
for _ in range(200): vision.far_band_mask(g, 0.0, 0.85, 1.2)
t2 = time.perf_counter()
for _ in range(200): vision.line_mask(g, 0.0, 1.2)
t3 = time.perf_counter()
print(f"near320 {(t1-t)/200*1e3:.2f} ms  farband640 {(t2-t1)/200*1e3:.2f} ms  full640 {(t3-t2)/200*1e3:.2f} ms")
