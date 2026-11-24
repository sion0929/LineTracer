# Perception bench (2026-11-24 16:21:46)

- recordings: line_track (28 frames, 7 hill)
- pitch mode `rest` (REST_PITCH 0.33 deg; on a slope REST_PITCH and x_max 0.5 m as in the node), OpenCV threads 1, latency = plugin.process() only
- IoU / P / R: BEV line vs true visible line inside the plugin's valid mask; cover: true line cells inside valid; IoU seg: vs the true image mask warped with the same bev_warp
- lateral error p50 / p95 [cm] at x = 0.35 / 0.50 / 0.70 m; det: route probes with a detected run within 10 cm

## Summary

| plugin | split | frames | ms mean | ms p95 | IoU | P | R | cover | IoU seg | lat 0.35 p50/p95 | lat 0.50 p50/p95 | lat 0.70 p50/p95 | lat all p95 | det |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| rule | ALL | 28 | 0.30 | 0.35 | 0.296 | 0.398 | 0.534 | 0.813 | 0.664 | 0.2 / 1.0 | 0.4 / 1.8 | 0.3 / 2.3 | 1.7 | 91.7% |
| rule | flat | 21 | 0.31 | 0.36 | 0.292 | 0.389 | 0.540 | 0.894 | 0.632 | 0.2 / 0.4 | 0.3 / 2.9 | 0.3 / 1.8 | 1.4 | 100.0% |
| rule | hill | 7 | 0.29 | 0.30 | 0.326 | 0.489 | 0.496 | 0.505 | 0.957 | 0.7 / 1.1 | 1.0 / 1.7 | 1.2 / 1.6 | 1.7 | 75.0% |
| rule_plane | ALL | 28 | 2.83 | 3.65 | 0.324 | 0.422 | 0.581 | 0.738 | 0.623 | 0.2 / 0.5 | 0.3 / 1.6 | 0.3 / 1.8 | 1.2 | 86.7% |
| rule_plane | flat | 21 | 2.63 | 3.06 | 0.303 | 0.398 | 0.561 | 0.861 | 0.627 | 0.2 / 0.4 | 0.3 / 2.9 | 0.3 / 1.8 | 1.4 | 100.0% |
| rule_plane | hill | 7 | 3.44 | 3.84 | 0.714 | 0.844 | 0.823 | 0.270 | 0.565 | 0.3 / 0.9 | 0.2 / 0.5 | - / - | 0.8 | 60.0% |
| rule_hill | ALL | 28 | 1.44 | 1.72 | 0.310 | 0.417 | 0.545 | 0.808 | 0.606 | 0.2 / 0.8 | 0.3 / 2.4 | 0.2 / 1.6 | 1.2 | 83.3% |
| rule_hill | flat | 21 | 1.36 | 1.47 | 0.291 | 0.387 | 0.540 | 0.887 | 0.630 | 0.2 / 0.4 | 0.3 / 2.9 | 0.3 / 1.8 | 1.4 | 100.0% |
| rule_hill | hill | 7 | 1.68 | 1.81 | 0.508 | 0.804 | 0.579 | 0.508 | 0.390 | 0.3 / 1.0 | 0.1 / 0.1 | 0.1 / 0.1 | 1.0 | 50.0% |
| dl | skipped: NotImplementedError: DlPerception.load: plug in the segmentation model here | | | | | | | | | | | | | |

## Per map (ALL frames)

| map | frames | rule IoU / lat p95 / det | rule_plane IoU / lat p95 / det | rule_hill IoU / lat p95 / det |
|---|---|---|---|---|
| line_track | 28 | 0.296 / 1.7 / 92% | 0.324 / 1.2 / 87% | 0.310 / 1.2 / 83% |

## Per scene type (ALL maps)

| type | frames | rule IoU / lat p95 / det | rule_plane IoU / lat p95 / det | rule_hill IoU / lat p95 / det |
|---|---|---|---|---|
| corner | 6 | 0.113 / 1.1 / 100% | 0.123 / 1.1 / 100% | 0.113 / 1.1 / 100% |
| crossing | 5 | 0.380 / 3.9 / 100% | 0.380 / 3.9 / 100% | 0.380 / 3.9 / 100% |
| hill | 7 | 0.326 / 1.7 / 75% | 0.714 / 0.8 / 60% | 0.508 / 1.0 / 50% |
| loop | 4 | 0.286 / 0.4 / 100% | 0.286 / 0.4 / 100% | 0.286 / 0.4 / 100% |
| straight | 6 | 0.598 / 0.4 / 100% | 0.598 / 0.4 / 100% | 0.605 / 0.4 / 100% |
