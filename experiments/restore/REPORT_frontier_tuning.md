# Frontier restoration tuning

Use **R3 with directional seeds, a 0.90 ridge threshold, a 12 cm cap, and a 0.98 mean-probability gate** as the offline candidate for `restore_mode=map`. It produced **96.96% precision**, **57.02% recall within its reach**, and **1.83% wrong patches (144/7,880)**. R5 with the same settings had the highest fixed 40 cm recall under 2%, leading R3 by 0.09 percentage points. R3 had higher precision and three fewer wrong patches, so it is the precision-first choice.

R3’s fixed 40 cm recall was **21.24%**. It drew at least one unseen point on **58.59%** of patches. Of the patches where it drew anything, **3.12%** contained a wrong chain. Across all patches, **3.21%** contained at least one point farther than 3 cm from the true route. The 2% criterion uses all patches and requires at least 5 cm of off-route output on one chain.

For a more conservative operating point, R3 with the same seed and threshold, a 10 cm cap, and a 0.99 mean gate produced **98.29% precision**, **0.27% wrong patches**, and **16.57% fixed 40 cm recall**.

## Results

The table shows the original baselines, selected controls, and the highest fixed-target recall under 2% at each eligible intermediate reach. The 20 cm rows show the lowest wrong-patch rate at that reach. Mean gate 0.00 means off. The complete grid, with counts, is in [frontier_sweep.csv](frontier_sweep.csv).

| Model | Seed | Threshold | Length (m) | Mean gate | Precision | Recall | Fixed 40 cm recall | Wrong patches | Drawn on | Any >3 cm |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| R3 | forward | 0.60 | 0.40 | 0.00 | 80.06% | 49.45% | 49.45% | 20.01% (1577) | 58.16% | 21.64% |
| R3 | direction | 0.60 | 0.20 | 0.99 | 93.92% | 42.89% | 24.60% | 4.70% (370) | 44.84% | 5.89% |
| R3 | direction | 0.90 | 0.12 | 0.00 | 95.88% | 61.34% | 22.77% | 2.53% (199) | 63.69% | 4.40% |
| R3 | direction | 0.90 | 0.12 | 0.98 | 96.96% | 57.02% | 21.24% | 1.83% (144) | 58.59% | 3.21% |
| R3 | direction | 0.90 | 0.10 | 0.00 | 97.03% | 62.80% | 18.72% | 0.90% (71) | 63.68% | 2.88% |
| R3 | direction | 0.90 | 0.10 | 0.99 | 98.29% | 55.49% | 16.57% | 0.27% (21) | 55.57% | 1.73% |
| R3 | forward | 0.90 | 0.10 | 0.00 | 96.94% | 58.73% | 16.78% | 0.85% (67) | 57.78% | 2.70% |
| R5 | forward | 0.60 | 0.40 | 0.00 | 79.70% | 49.48% | 49.48% | 20.28% (1598) | 58.54% | 21.83% |
| R5 | forward | 0.70 | 0.20 | 0.99 | 93.61% | 42.36% | 23.82% | 4.61% (363) | 42.91% | 5.90% |
| R5 | direction | 0.90 | 0.12 | 0.00 | 95.58% | 61.29% | 22.76% | 2.73% (215) | 63.87% | 4.49% |
| R5 | direction | 0.90 | 0.12 | 0.98 | 96.74% | 57.26% | 21.33% | 1.87% (147) | 58.82% | 3.38% |
| R5 | direction | 0.90 | 0.10 | 0.00 | 96.78% | 62.76% | 18.72% | 1.08% (85) | 63.85% | 2.97% |
| R5 | direction | 0.90 | 0.10 | 0.99 | 98.18% | 55.90% | 16.70% | 0.30% (24) | 55.94% | 1.74% |
| R5 | forward | 0.90 | 0.10 | 0.00 | 96.72% | 58.70% | 16.78% | 1.03% (81) | 58.05% | 2.87% |

The R3 baseline reproduced all four published counts (54,021 correct of 67,478 predicted points and 213,946 covered of 432,669 target pixels). Only the 10 and 12 cm caps had settings below 2% for either model. At 12 cm, the 0.98 mean gate took R3 from 2.53% to 1.83% wrong patches and raised precision by 1.08 percentage points.

## Method

- Scored all 7,880 robot-centric 128×128 patches in `/home/rvlab/restore_data/val.npz`. The two `x` channels represent the seen line and seen area. `y` is the rasterized route and `unseen` marks true route pixels outside the seen area. The patch pitch is 1.25 cm.
- Cached float32 sigmoid probabilities after one CPU ONNX pass per model. Both models used the existing evaluator’s ONNX Runtime CPU provider with four intra-op threads. OpenCV and OMP were limited to four threads and jobs ran at nice +10.
- Used the checked-out `Restorer.extend_ridge` for every distinct seed and threshold. Shorter walks are prefixes of the 40 cm walk because the length cap is applied after the walker selects its path. Both seed rules used 8-connected components of the seen-line channel within the 5×5 dilation of unseen space. Seeds needed to be strictly ahead of 0.2 m. `forward` uses the first row-major pixel at the greatest forward position. `direction` projects candidate pixels along the vector from the centroid of nearby interior seen-line pixels to the frontier component’s eligible-pixel centroid. Interior pixels are outside the dilated unseen area and within 12 cm of that centroid. It falls back to `forward` if that vector is unavailable or zero.
- The mean gate rejects a whole length-capped chain when the mean probability at its sampled points falls below the gate, including points in seen space. A gate of 0 means disabled. The grid was 2 seed rules × 4 ridge thresholds (0.60, 0.70, 0.80, 0.90) × 4 length caps (0.10 to 0.40 m at 0.10 m intervals) × 7 mean gates (off, 0.75, 0.85, 0.90, 0.95, 0.98, 0.99) per model.
- **Precision** pools unique returned unseen pixels across patches, with success within 2 cm of any true route pixel. **Recall** pools unseen true-route pixels within the length cap’s Euclidean radius of a seed, counting those within 2 cm of a predicted unseen pixel. **Fixed 40 cm recall** uses the same target for all rows, defined by the 40 cm radius of the original forward seeds. Rejected chains do not shrink either target.
- **Wrong patches** is the fraction of all 7,880 patches with at least one accepted chain accumulating at least 5 cm of sampled step lengths whose endpoint is unseen and farther than 3 cm from the true route. The first step is measured from the frontier seed. Multiple bad segments on the same chain count toward the 5 cm total. **Drawn on** is the fraction with any unseen output. **Any >3 cm** counts patches with at least one such bad endpoint, even if the chain was too short to meet the 5 cm criterion.
- This patch-level evaluation scores chain centers before odom rasterization and the map’s 3×3 stamp. It measures proximity to the route raster, so a jump onto a nearby valid segment can still score as correct. It does not measure planner choice, repeated errors during driving, or official-track completion. The same validation set was used for selection and measurement.

## Reproduce

Run from the workspace root. Each cache is approximately 493 MiB and carries model and validation SHA-256 hashes.

```bash
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
py=/home/rvlab/miniforge3/envs/lf_b_dl/bin/python
nice -n 10 "$py" experiments/restore/tune_frontier.py \
  --model experiments/restore/models/restore_r3.onnx \
  --cache /tmp/26rp_frontier_r3.npz --scores /tmp/26rp_frontier_r3_scores.npz
nice -n 10 "$py" experiments/restore/tune_frontier.py \
  --model /home/rvlab/restore_data/runs/R5_w24_160k/restore.onnx \
  --cache /tmp/26rp_frontier_r5.npz --scores /tmp/26rp_frontier_r5_scores.npz
```

| Input | SHA-256 |
|---|---|
| Validation NPZ | `9c6bfd1b30ce2da5ceac8dba6d0706def7d60fa3e254bfdbe8eef47204dee833` |
| R3 ONNX | `fe81a0f40b2d2a80e83e6aabbb02ee7658d58f648fa60b16a479436261a12683` |
| R5 ONNX | `c3371ab7a82179213a6c325aeb0eb7ffc50caff570cdf1afeebf3a9e01fe318f` |

The intermediate sweep used the same grid at 0.12, 0.14, 0.16 and 0.18 m, for 448 settings per model overall. Repeat each command with `--intermediate-lengths` and a distinct `--scores` path to reproduce these rows.

## Proposed unified diff

This diff targets the current workspace files and was checked with `git apply --check`. The revised code was also loaded in memory and matched seed and chain output on 100 sampled validation patches. Both models passed that check and the map-stamping smoke test. The proposed test-fixture adjustment keeps the positive test ridge above the new mean gate. No source changes were applied.

To reproduce this setting in the node, use `restore_mode:=map`, the R3 ONNX as `restore_model`, `restore_thresh:=0.90`, and `restore_len:=0.12`. The diff enforces the tuned lower threshold bound and upper length bound in map mode even with the original ROS defaults. Closed-loop driving was not run.

```diff
--- a/src/line_follower/test/test_pipeline_units.py
+++ b/src/line_follower/test/test_pipeline_units.py
@@ -104,7 +104,7 @@
 
     def route(lm, pose):
         prob = np.zeros((128, 128), np.float32)
-        prob[:, 63:65] = 0.95                    # robot-centric patch column 64 = y 0
+        prob[:, 63:65] = 0.999                   # robot-centric patch column 64 = y 0
         return prob
     r.route = route
     lm = LineMap()
--- a/src/line_follower/line_follower/line_map.py
+++ b/src/line_follower/line_follower/line_map.py
@@ -45,6 +45,10 @@
         never-seen ground ("frontier"), follow the learned route (restore.Restorer) for at most
         `length` m and draw that thin route into self.restored. Refreshed within radius of the
         robot on every call; cells the camera has seen are never restored."""
+        from .restore import PRES
+
+        thresh = max(thresh, 0.90)
+        length = min(length, 0.12)
         if self.restored is None:
             self.restored = np.zeros((SIZE, SIZE), np.uint8)
         px, py, th = pose
@@ -60,18 +64,30 @@
         n, lab = cv2.connectedComponents(front.astype(np.uint8), connectivity=8)
         ly, lx = np.nonzero(line > 0)
         lxr, lyr = restorer.to_robot(lx, ly)
+        interior = (line > 0) & (seen > 0) & ~unseen_near
+        interior_rc = np.column_stack(np.nonzero(interior))
         c, s = math.cos(th), math.sin(th)
         for k in range(1, n):
             rows, cols = np.nonzero(lab == k)
             xs, ys = restorer.to_robot(cols, rows)
-            far = int(np.argmax(xs))                 # the frontier end farthest ahead
-            start = (float(xs[far]), float(ys[far]))
-            if start[0] < 0.2:
-                continue                             # behind / beside the robot: nothing to restore
+            eligible = xs > 0.2
+            if not eligible.any():
+                continue
+            xs, ys = xs[eligible], ys[eligible]
+            pixels = np.column_stack((rows[eligible], cols[eligible]))
+            center = pixels.mean(axis=0)
+            offsets = center - interior_rc
+            local = offsets[np.sum(offsets ** 2, axis=1) < (0.12 / PRES) ** 2]
+            pick = int(np.argmax(xs))
+            if len(local):
+                outward = local.mean(axis=0)
+                if np.linalg.norm(outward) > 0:
+                    pick = int(np.argmax((pixels - center) @ outward))
+            start = (float(xs[pick]), float(ys[pick]))
             near = np.hypot(lxr - start[0], lyr - start[1]) < 0.12
             order = np.argsort(-np.hypot(lxr[near] - start[0], lyr[near] - start[1]))
             known = list(zip(lxr[near][order], lyr[near][order]))       # farthest first, start last
-            chain = restorer.extend_ridge(prob, start, known, length, thresh)
+            chain = restorer.extend_ridge(prob, start, known, length, thresh, min_mean=0.98)
             for x, y in chain:
                 X = px + x * c - y * s
                 Y = py + x * s + y * c
--- a/src/line_follower/line_follower/restore.py
+++ b/src/line_follower/line_follower/restore.py
@@ -73,10 +73,11 @@
     def to_robot(col, row):
         return X1 - (np.asarray(row) + 0.5) * PRES, Y0 - (np.asarray(col) + 0.5) * PRES
 
-    def extend_ridge(self, prob, start, known, length=0.4, thresh=0.6):
+    def extend_ridge(self, prob, start, known, length=0.4, thresh=0.6, *, min_mean=0.0):
         """Longest walk from `start` through the confident part of the prediction (any angle:
         acute corners and U-turns too), avoiding the already known path `known` (robot frame).
-        Returns robot-frame points ~2 cm apart, at most `length` long."""
+        Returns robot-frame points ~2 cm apart, at most `length` long.
+        Rejects the chain if its sampled mean probability is below `min_mean`."""
         from collections import deque
         m = prob > thresh
         if not m.any():
@@ -130,6 +131,10 @@
                 break
             out.append((x, y))
             last = (x, y)
+        if out and min_mean > 0:
+            cols, rows = zip(*(px(q) for q in out))
+            if float(prob[rows, cols].mean()) < min_mean:
+                return []
         return out
 
     def extend(self, prob, start, heading, length=0.6, step=0.02, thresh=0.5):
```
