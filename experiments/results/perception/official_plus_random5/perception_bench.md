# Perception bench (2026-11-24 16:21:27)

- recordings: line_track (50 frames, 10 hill), normal_5009_m (39 frames, 25 hill), hard_9002_m (38 frames, 19 hill), normal_1136_m (40 frames, 26 hill), easy_1105 (40 frames, 13 hill), hard_7012 (40 frames, 21 hill)
- pitch mode `rest` (REST_PITCH 0.33 deg; on a slope REST_PITCH and x_max 0.5 m as in the node), OpenCV threads 1, latency = plugin.process() only
- IoU / P / R: BEV line vs true visible line inside the plugin's valid mask; cover: true line cells inside valid; IoU seg: vs the true image mask warped with the same bev_warp
- lateral error p50 / p95 [cm] at x = 0.35 / 0.50 / 0.70 m; det: route probes with a detected run within 10 cm

- random maps: seed 1539359436 (general200,official_like50,hills60,stress50), sample in `experiments/results/perception/official_plus_random5/maps`

## Summary

| plugin | split | frames | ms mean | ms p95 | IoU | P | R | cover | IoU seg | lat 0.35 p50/p95 | lat 0.50 p50/p95 | lat 0.70 p50/p95 | lat all p95 | det |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| rule | ALL | 247 | 0.31 | 0.37 | 0.365 | 0.510 | 0.562 | 0.753 | 0.860 | 0.3 / 2.0 | 0.3 / 2.9 | 0.3 / 2.6 | 2.5 | 86.1% |
| rule | flat | 133 | 0.31 | 0.37 | 0.370 | 0.529 | 0.551 | 0.904 | 0.844 | 0.3 / 0.6 | 0.3 / 1.7 | 0.3 / 1.8 | 1.0 | 98.7% |
| rule | hill | 114 | 0.30 | 0.37 | 0.354 | 0.473 | 0.586 | 0.548 | 0.890 | 0.5 / 4.0 | 0.7 / 3.4 | 0.5 / 5.1 | 3.9 | 74.4% |
| rule_plane | ALL | 247 | 2.86 | 3.77 | 0.415 | 0.577 | 0.598 | 0.643 | 0.771 | 0.3 / 1.2 | 0.3 / 1.0 | 0.3 / 1.4 | 1.1 | 77.8% |
| rule_plane | flat | 133 | 2.63 | 3.17 | 0.395 | 0.560 | 0.573 | 0.804 | 0.852 | 0.3 / 0.6 | 0.3 / 0.9 | 0.3 / 0.8 | 0.8 | 95.0% |
| rule_plane | hill | 114 | 3.13 | 3.92 | 0.468 | 0.618 | 0.659 | 0.425 | 0.612 | 0.4 / 2.9 | 0.2 / 1.0 | 0.2 / 2.8 | 2.3 | 61.8% |
| rule_hill | ALL | 247 | 1.47 | 1.90 | 0.382 | 0.544 | 0.563 | 0.738 | 0.793 | 0.3 / 1.9 | 0.2 / 2.0 | 0.2 / 1.5 | 2.0 | 81.3% |
| rule_hill | flat | 133 | 1.40 | 1.80 | 0.369 | 0.526 | 0.552 | 0.898 | 0.835 | 0.3 / 0.6 | 0.3 / 1.7 | 0.3 / 1.8 | 1.0 | 98.2% |
| rule_hill | hill | 114 | 1.54 | 2.03 | 0.416 | 0.588 | 0.587 | 0.521 | 0.700 | 0.5 / 4.5 | 0.2 / 2.1 | 0.1 / 0.5 | 3.7 | 65.5% |
| dl | skipped: NotImplementedError: DlPerception.load: plug in the segmentation model here | | | | | | | | | | | | | |

## Per map (ALL frames)

| map | frames | rule IoU / lat p95 / det | rule_plane IoU / lat p95 / det | rule_hill IoU / lat p95 / det |
|---|---|---|---|---|
| easy_1105 | 40 | 0.534 / 1.0 / 95% | 0.602 / 0.8 / 90% | 0.527 / 0.8 / 90% |
| hard_7012 | 40 | 0.368 / 2.4 / 77% | 0.438 / 0.8 / 62% | 0.392 / 1.6 / 67% |
| hard_9002_m | 38 | 0.348 / 5.9 / 82% | 0.376 / 2.3 / 76% | 0.363 / 5.5 / 82% |
| line_track | 50 | 0.325 / 2.0 / 94% | 0.355 / 1.1 / 83% | 0.332 / 1.6 / 89% |
| normal_1136_m | 40 | 0.292 / 2.2 / 80% | 0.366 / 1.3 / 70% | 0.322 / 2.0 / 74% |
| normal_5009_m | 39 | 0.340 / 4.3 / 83% | 0.391 / 3.4 / 80% | 0.381 / 4.3 / 80% |

## Per scene type (ALL maps)

| type | frames | rule IoU / lat p95 / det | rule_plane IoU / lat p95 / det | rule_hill IoU / lat p95 / det |
|---|---|---|---|---|
| corner | 46 | 0.197 / 2.4 / 94% | 0.229 / 1.7 / 92% | 0.196 / 2.4 / 94% |
| crossing | 32 | 0.417 / 0.7 / 100% | 0.436 / 0.5 / 92% | 0.416 / 0.7 / 99% |
| hill | 112 | 0.357 / 4.0 / 75% | 0.467 / 2.4 / 62% | 0.418 / 3.8 / 66% |
| loop | 19 | 0.290 / 2.9 / 100% | 0.303 / 1.2 / 94% | 0.290 / 2.9 / 100% |
| straight | 38 | 0.586 / 0.4 / 98% | 0.596 / 0.4 / 98% | 0.596 / 0.4 / 98% |
