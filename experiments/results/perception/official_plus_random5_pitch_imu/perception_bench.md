# Perception bench (2026-11-24 16:21:40)

- recordings: line_track (50 frames, 10 hill), easy_1105 (40 frames, 13 hill), hard_7012 (40 frames, 21 hill), hard_9002_m (38 frames, 19 hill), normal_1136_m (40 frames, 26 hill), normal_5009_m (39 frames, 25 hill)
- pitch mode `imu` (REST_PITCH 0.33 deg; on a slope REST_PITCH and x_max 0.5 m as in the node), OpenCV threads 1, latency = plugin.process() only
- IoU / P / R: BEV line vs true visible line inside the plugin's valid mask; cover: true line cells inside valid; IoU seg: vs the true image mask warped with the same bev_warp
- lateral error p50 / p95 [cm] at x = 0.35 / 0.50 / 0.70 m; det: route probes with a detected run within 10 cm

## Summary

| plugin | split | frames | ms mean | ms p95 | IoU | P | R | cover | IoU seg | lat 0.35 p50/p95 | lat 0.50 p50/p95 | lat 0.70 p50/p95 | lat all p95 | det |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| rule | ALL | 247 | 0.32 | 0.40 | 0.468 | 0.610 | 0.668 | 0.753 | 0.847 | 0.3 / 2.3 | 0.3 / 2.9 | 0.3 / 1.1 | 2.2 | 85.9% |
| rule | flat | 133 | 0.32 | 0.40 | 0.532 | 0.686 | 0.703 | 0.904 | 0.833 | 0.3 / 0.6 | 0.3 / 1.2 | 0.2 / 0.9 | 0.9 | 98.7% |
| rule | hill | 114 | 0.31 | 0.40 | 0.355 | 0.471 | 0.591 | 0.547 | 0.872 | 0.5 / 4.0 | 0.7 / 3.4 | 0.5 / 1.7 | 3.3 | 74.0% |
| rule_plane | ALL | 247 | 2.89 | 3.74 | 0.522 | 0.678 | 0.694 | 0.650 | 0.774 | 0.3 / 1.3 | 0.3 / 0.9 | 0.2 / 1.1 | 1.1 | 77.8% |
| rule_plane | flat | 133 | 2.66 | 3.18 | 0.545 | 0.703 | 0.708 | 0.818 | 0.842 | 0.3 / 0.6 | 0.3 / 0.8 | 0.2 / 0.7 | 0.8 | 95.0% |
| rule_plane | hill | 114 | 3.16 | 4.07 | 0.466 | 0.617 | 0.656 | 0.422 | 0.636 | 0.4 / 2.9 | 0.2 / 0.9 | 0.3 / 2.8 | 2.3 | 61.8% |
| rule_hill | ALL | 247 | 1.45 | 1.83 | 0.492 | 0.650 | 0.670 | 0.735 | 0.784 | 0.3 / 2.0 | 0.2 / 2.0 | 0.2 / 0.9 | 1.9 | 80.0% |
| rule_hill | flat | 133 | 1.40 | 1.70 | 0.530 | 0.682 | 0.704 | 0.897 | 0.825 | 0.3 / 0.7 | 0.3 / 1.2 | 0.2 / 0.9 | 0.9 | 98.2% |
| rule_hill | hill | 114 | 1.52 | 1.93 | 0.410 | 0.574 | 0.589 | 0.516 | 0.697 | 0.4 / 4.4 | 0.2 / 2.1 | 0.2 / 0.6 | 3.2 | 63.0% |
| dl | skipped: NotImplementedError: DlPerception.load: plug in the segmentation model here | | | | | | | | | | | | | |

## Per map (ALL frames)

| map | frames | rule IoU / lat p95 / det | rule_plane IoU / lat p95 / det | rule_hill IoU / lat p95 / det |
|---|---|---|---|---|
| easy_1105 | 40 | 0.646 / 1.0 / 95% | 0.680 / 0.7 / 91% | 0.639 / 0.7 / 88% |
| hard_7012 | 40 | 0.489 / 2.5 / 77% | 0.584 / 0.7 / 65% | 0.525 / 1.4 / 67% |
| hard_9002_m | 38 | 0.472 / 5.5 / 82% | 0.504 / 2.3 / 76% | 0.485 / 5.6 / 81% |
| line_track | 50 | 0.448 / 1.6 / 94% | 0.483 / 0.8 / 81% | 0.462 / 0.9 / 88% |
| normal_1136_m | 40 | 0.392 / 2.0 / 80% | 0.461 / 1.2 / 70% | 0.436 / 2.0 / 72% |
| normal_5009_m | 39 | 0.375 / 2.9 / 82% | 0.417 / 3.4 / 77% | 0.414 / 3.7 / 79% |

## Per scene type (ALL maps)

| type | frames | rule IoU / lat p95 / det | rule_plane IoU / lat p95 / det | rule_hill IoU / lat p95 / det |
|---|---|---|---|---|
| corner | 46 | 0.361 / 1.1 / 94% | 0.374 / 1.1 / 90% | 0.356 / 1.1 / 94% |
| crossing | 32 | 0.607 / 0.6 / 100% | 0.619 / 0.5 / 93% | 0.607 / 0.6 / 99% |
| hill | 112 | 0.362 / 3.3 / 74% | 0.471 / 2.4 / 62% | 0.418 / 3.3 / 63% |
| loop | 19 | 0.502 / 2.2 / 100% | 0.512 / 0.8 / 94% | 0.501 / 2.2 / 100% |
| straight | 38 | 0.641 / 0.4 / 98% | 0.652 / 0.4 / 98% | 0.648 / 0.4 / 98% |
