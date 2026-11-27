# Far-band ground placement error

Use a **1.60 m far-band end** on the evaluated flat ground. `farband_w8h2` (thinned) meets both criteria in every bin up to that cut.

The gate uses unrounded median ≤2 cm and P≤3 cm ≥70% in every successive bin from 0.85 m. Among learned variants with equal reach, the recommendation favors the highest worst-bin P≤3 cm within that reach. Recall is reported separately.

| Detector | Raw end (m) | Thinned end (m) |
|---|---:|---:|
| farband_w8h2 | 1.60 | 1.60 |
| farband_w8h2_hill | 1.55 | 1.60 |
| threshold | 1.45 | 1.25 |

## Method

- Evaluated all 1,210 stored flat-ground frames from the last ten sorted map names. The empty capture contributed zero frames. This evaluation covers flat ground at zero pitch.
- Both ONNX models ran on the OpenCV CPU backend. RGB columns were downscaled with area interpolation and normalized by 255. Logits >0 were upscaled by nearest neighbor. The threshold baseline used RGB-to-gray and gray < min(125, 0.62 × strip median). The full strip was the available proxy for bottom-row ground brightness and can include non-ground pixels.
- All masks used the 640×480 image layout with the strip in rows 256–319 and `far_band_mask(..., 0.0, 0.85, 1.6, line_img=...)`. Predicted 5 mm BEV cells were converted through `bev_to_robot` and the stored pose into world XY. The valid grid spans 0.860–1.595 m after border erosion. Bins include their lower edge and exclude their upper edge, with the final bin closed at 1.60 m.
- Sidecar checks reprojected the centerline and markings using the capture's 2×2 subpixel grid and 1 cm half-width at up to 12 evenly spaced poses per map. A sidecar required at least three informative samples (nonempty union), pooled positive-support IoU ≥0.90 and minimum informative-frame IoU ≥0.75. Unverified maps used the stored positive-coverage pixels (y > 0), binarized before the same BEV warp.
- For verified sidecars, forward error is measured to the closest centerline or marking sample on the world polyline (spacing ≤2.5 mm). Recall targets valid cells within 1 cm of that geometry. For label fallback, forward error and recall use the projected positive-coverage cells as reference, so errors there measure distance to rasterized line support.
- Distances are Euclidean within a frame. Nearest neighbors may cross bin boundaries. Precision bins by predicted x, recall by true x. Cell counts and distances are pooled across frames. A prediction with no label reference in the band has infinite distance. A missing prediction contributes zero recall hits. Quantiles use the inverse empirical CDF. Thinning uses `thin_across(mask, max_run=16, keep=1)` on each boolean BEV prediction.

## Sidecar audit

| Map | Frames | Informative checks | Pooled IoU | Min IoU | Ground truth |
|---|---:|---:|---:|---:|---|
| tile_050 | 150 | 9 | 0.139 | 0.000 | Projected labels |
| tile_051 | 150 | 11 | 0.200 | 0.000 | Projected labels |
| tile_052 | 150 | 6 | 0.330 | 0.153 | Projected labels |
| tile_053 | 150 | 8 | 0.271 | 0.000 | Projected labels |
| tile_054 | 0 | 0 | — | — | Empty capture |
| tile_055 | 150 | 12 | 0.254 | 0.149 | Projected labels |
| tile_056 | 137 | 5 | 0.010 | 0.000 | Projected labels |
| tile_057 | 150 | 7 | 0.087 | 0.000 | Projected labels |
| tile_058 | 150 | 6 | 0.064 | 0.000 | Projected labels |
| tile_059 | 23 | 11 | 0.122 | 0.008 | Projected labels |

## Results

### farband_w8h2 (raw)

| Forward bin (m) | P≤2 cm | P≤3 cm | Median (cm) | P90 (cm) | R≤3 cm | Pred cells | True cells |
|---|---:|---:|---:|---:|---:|---:|---:|
| 0.85–0.95 | 88.0% | 94.7% | 0.00 | 2.50 | 99.9% | 166,949 | 101,985 |
| 0.95–1.05 | 81.8% | 91.2% | 0.50 | 3.00 | 99.2% | 209,926 | 122,663 |
| 1.05–1.15 | 77.6% | 85.7% | 0.50 | 3.81 | 98.5% | 186,491 | 110,845 |
| 1.15–1.25 | 75.7% | 83.5% | 0.50 | 4.27 | 98.0% | 163,050 | 104,183 |
| 1.25–1.35 | 73.9% | 80.9% | 0.50 | 4.72 | 93.7% | 131,911 | 91,781 |
| 1.35–1.45 | 75.6% | 82.7% | 0.50 | 5.00 | 88.0% | 117,275 | 92,316 |
| 1.45–1.55 | 77.7% | 84.5% | 0.50 | 4.61 | 67.5% | 91,145 | 96,511 |
| 1.55–1.60 | 80.8% | 87.0% | 0.50 | 3.91 | 58.4% | 37,432 | 43,443 |

### farband_w8h2 (thinned)

| Forward bin (m) | P≤2 cm | P≤3 cm | Median (cm) | P90 (cm) | R≤3 cm | Pred cells | True cells |
|---|---:|---:|---:|---:|---:|---:|---:|
| 0.85–0.95 | 91.7% | 96.3% | 0.00 | 2.00 | 99.6% | 96,808 | 101,985 |
| 0.95–1.05 | 85.6% | 93.4% | 0.00 | 2.50 | 98.7% | 133,174 | 122,663 |
| 1.05–1.15 | 81.9% | 88.6% | 0.00 | 3.50 | 98.3% | 127,413 | 110,845 |
| 1.15–1.25 | 80.7% | 86.3% | 0.00 | 3.91 | 97.9% | 117,590 | 104,183 |
| 1.25–1.35 | 79.4% | 84.4% | 0.50 | 4.24 | 91.8% | 101,536 | 91,781 |
| 1.35–1.45 | 82.0% | 87.5% | 0.50 | 4.00 | 85.3% | 89,258 | 92,316 |
| 1.45–1.55 | 82.6% | 88.8% | 0.50 | 3.50 | 64.6% | 74,409 | 96,511 |
| 1.55–1.60 | 83.7% | 89.8% | 0.00 | 3.04 | 57.6% | 28,987 | 43,443 |

### farband_w8h2_hill (raw)

| Forward bin (m) | P≤2 cm | P≤3 cm | Median (cm) | P90 (cm) | R≤3 cm | Pred cells | True cells |
|---|---:|---:|---:|---:|---:|---:|---:|
| 0.85–0.95 | 90.2% | 95.7% | 0.00 | 2.00 | 99.6% | 158,436 | 101,985 |
| 0.95–1.05 | 84.0% | 92.2% | 0.50 | 2.92 | 96.5% | 196,852 | 122,663 |
| 1.05–1.15 | 79.7% | 87.1% | 0.50 | 3.50 | 91.8% | 168,296 | 110,845 |
| 1.15–1.25 | 78.3% | 85.5% | 0.50 | 4.00 | 87.3% | 148,504 | 104,183 |
| 1.25–1.35 | 72.2% | 78.8% | 0.50 | 5.50 | 81.9% | 130,789 | 91,781 |
| 1.35–1.45 | 72.2% | 78.7% | 0.50 | 6.00 | 77.5% | 121,148 | 92,316 |
| 1.45–1.55 | 68.3% | 74.8% | 0.50 | 7.16 | 69.9% | 117,834 | 96,511 |
| 1.55–1.60 | 63.8% | 68.6% | 0.71 | 8.20 | 65.3% | 49,971 | 43,443 |

### farband_w8h2_hill (thinned)

| Forward bin (m) | P≤2 cm | P≤3 cm | Median (cm) | P90 (cm) | R≤3 cm | Pred cells | True cells |
|---|---:|---:|---:|---:|---:|---:|---:|
| 0.85–0.95 | 94.1% | 97.0% | 0.00 | 1.50 | 97.5% | 86,108 | 101,985 |
| 0.95–1.05 | 91.1% | 96.2% | 0.00 | 2.00 | 93.5% | 112,118 | 122,663 |
| 1.05–1.15 | 85.3% | 90.8% | 0.00 | 3.00 | 88.1% | 111,233 | 110,845 |
| 1.15–1.25 | 82.7% | 88.7% | 0.00 | 3.50 | 84.1% | 104,182 | 104,183 |
| 1.25–1.35 | 79.6% | 84.9% | 0.00 | 4.50 | 77.9% | 89,092 | 91,781 |
| 1.35–1.45 | 81.7% | 86.6% | 0.00 | 4.27 | 73.5% | 80,521 | 92,316 |
| 1.45–1.55 | 78.5% | 84.1% | 0.50 | 5.50 | 65.5% | 77,705 | 96,511 |
| 1.55–1.60 | 73.0% | 78.4% | 0.50 | 6.50 | 64.6% | 33,294 | 43,443 |

### threshold (raw)

| Forward bin (m) | P≤2 cm | P≤3 cm | Median (cm) | P90 (cm) | R≤3 cm | Pred cells | True cells |
|---|---:|---:|---:|---:|---:|---:|---:|
| 0.85–0.95 | 91.9% | 95.5% | 0.00 | 2.00 | 95.7% | 80,294 | 101,985 |
| 0.95–1.05 | 84.2% | 90.9% | 0.00 | 3.00 | 91.9% | 90,016 | 122,663 |
| 1.05–1.15 | 78.0% | 83.4% | 0.00 | 5.50 | 84.7% | 80,731 | 110,845 |
| 1.15–1.25 | 71.0% | 76.4% | 0.00 | 20.26 | 75.7% | 70,410 | 104,183 |
| 1.25–1.35 | 68.7% | 72.4% | 0.50 | 23.69 | 69.8% | 58,830 | 91,781 |
| 1.35–1.45 | 68.1% | 71.7% | 0.50 | 26.61 | 63.5% | 50,486 | 92,316 |
| 1.45–1.55 | 65.7% | 69.1% | 0.50 | 33.80 | 51.9% | 42,118 | 96,511 |
| 1.55–1.60 | 63.6% | 66.6% | 0.50 | 39.08 | 49.3% | 18,548 | 43,443 |

### threshold (thinned)

| Forward bin (m) | P≤2 cm | P≤3 cm | Median (cm) | P90 (cm) | R≤3 cm | Pred cells | True cells |
|---|---:|---:|---:|---:|---:|---:|---:|
| 0.85–0.95 | 91.0% | 94.8% | 0.00 | 2.00 | 95.3% | 58,316 | 101,985 |
| 0.95–1.05 | 82.1% | 89.7% | 0.00 | 3.20 | 91.1% | 72,825 | 122,663 |
| 1.05–1.15 | 76.3% | 81.9% | 0.00 | 21.26 | 84.1% | 62,714 | 110,845 |
| 1.15–1.25 | 69.6% | 73.5% | 0.00 | 27.21 | 73.9% | 52,578 | 104,183 |
| 1.25–1.35 | 67.2% | 69.4% | 0.50 | 28.15 | 67.4% | 44,848 | 91,781 |
| 1.35–1.45 | 65.0% | 67.5% | 0.50 | 31.66 | 61.2% | 38,701 | 92,316 |
| 1.45–1.55 | 60.7% | 63.5% | 0.50 | 38.55 | 49.8% | 33,326 | 96,511 |
| 1.55–1.60 | 56.9% | 59.8% | 1.00 | 50.36 | 48.5% | 14,185 | 43,443 |
