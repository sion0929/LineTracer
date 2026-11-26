import os, sys
os.environ["LF_X_FAR"] = sys.argv[1] if len(sys.argv) > 1 else "1.5"
from pathlib import Path
import cv2, numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/line_follower"))
from line_follower import vision, far_band
D = ROOT / "experiments/results/lead/res_compare"
m = far_band.load(str(ROOT / "experiments/farband/models/V2-Far640-Gz.onnx"))
rows = []
for n in ("tile_034_ex0", "tile_034_ex2", "tile_009_ex1", "tile_009_ex3"):
    bgr = cv2.imread(str(D / f"{n}.png")); g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    near, nv = vision.line_mask(g, 0.0, 0.85, scale=0.5)
    li = m.mask(bgr, 0.0)
    f1, fv = vision.far_band_mask(g, 0.0, 0.85, vision.X_FAR, line_img=li, thin=0.08)
    f0, _ = vision.far_band_mask(g, 0.0, 0.85, vision.X_FAR)          # threshold
    tiles = []
    for (l, lab) in ((np.maximum(near, f0), "near320 + far threshold"), (np.maximum(near, f1), "near320 + far MODEL")):
        t = cv2.cvtColor(np.where(np.maximum(nv, fv) > 0, l, 50).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        for d in (0.85, 1.2):
            r = int((vision.X_FAR - d) / vision.RES); cv2.line(t, (0, r), (t.shape[1], r), (0, 140, 255), 1)
        cv2.putText(t, lab, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 255), 1)
        tiles.append(t)
    ov = bgr.copy(); ov[li > 0] = (0, 0, 255)
    ov = cv2.resize(ov, (int(640 * tiles[0].shape[0] / 480), tiles[0].shape[0]))
    rows.append(np.hstack([ov] + tiles))
cv2.imwrite(str(D / "far_model_check.png"), np.vstack(rows))
