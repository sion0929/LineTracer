# Far-band GPU sweep

| Variant | Val precision | R 0.8–1.0 m | R 1.0–1.2 m | R 1.2–1.5 m | R 1.5–2.0 m | Selection F1 | Lab CPU ms, 2 threads |
|---|---:|---:|---:|---:|---:|---:|---:|
| Existing w8 h2, 14 epochs | 0.5255 | 0.9756 | 0.9061 | 0.8151 | 0.5281 | 0.6526 | NOT_RUN |
| w12 h2, 14 epochs | NOT_RUN | — | — | — | — | — | NOT_RUN |
| w16 h2, 14 epochs | NOT_RUN | — | — | — | — | — | NOT_RUN |
| w8 h2, 30 epochs | NOT_RUN | — | — | — | — | — | NOT_RUN |
| w10 h2, 14 epochs | NOT_RUN | — | — | — | — | — | NOT_RUN |

The baseline values come from epoch index 8 in the existing run's metrics. Selection F1 is the harmonic mean of precision and the mean recall across 1.0–1.2 m and 1.2–1.5 m. The trainer bins rows using flat-ground distances and measures recall after a 3×3 dilation. The fourth sweep variant uses width 10 to test a capacity point between widths 8 and 12.

CUDA initialization failed with “No CUDA GPUs are available,” and this session exposes no NVIDIA device nodes. The training launcher stopped at preflight. The requested Docker group command failed with “Cannot open audit interface,” and direct Docker access was denied at the socket.

The existing export passed a host OpenCV 4.13.0 smoke test and took 8.301 ms median over 100 timed calls with two threads after 20 warmup calls. The 10 ms lab budget requires measurement with OpenCV 4.6 in `line_follower_lab`.

Keep using `experiments/farband/runs/fb_w8_h2_hill/farband.onnx` pending the sweep. No model was promoted to `models/farband_best.onnx`.

The sweep launcher is `train_sweep.sh`. It uses the original trainer, both data directories, and the last ten unique map names for validation. The following command benchmarks the exports and the baseline from the repository mount:

```bash
sg docker -c "docker exec line_follower_lab bash -lc 'cd /ws && python3 experiments/farband/runs_sweep/benchmark.py experiments/farband/runs_sweep/*/farband.onnx experiments/farband/runs/fb_w8_h2_hill/farband.onnx'"
```
