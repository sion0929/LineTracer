# R6 frontier restoration

The GPU training entry point returned `CUDA is unavailable`. The NVIDIA device nodes are absent in this session. The requested run directory under `/home/rvlab/restore_data/runs` is outside the writable workspace. The lab container command failed with `Cannot open audit interface - aborting` through `sg docker`, and direct Docker access was denied at the socket.

| Model | Route IoU | Unseen recall | False rate | Chain precision | Chain recall | Lab OpenCV 4.6 CPU median / p95 | Local OpenCV 4.13 CPU median / p95 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| R3 w24 mix, epoch 22 | 0.750 | 89.10% | 10.14% | 80.06% | 49.45% | NOT_RUN | 15.10 / 20.91 ms |
| R5 w24 160k, epoch 24 | 0.735 | 91.09% | 11.96% | 79.70% | 49.48% | NOT_RUN | 14.77 / 17.16 ms |
| R6 w24 frontier | NOT_RUN | NOT_RUN | NOT_RUN | NOT_RUN | NOT_RUN | NOT_RUN | NOT_RUN |
| R6 w24 frontier + hard negatives | NOT_RUN | NOT_RUN | NOT_RUN | NOT_RUN | NOT_RUN | NOT_RUN | NOT_RUN |

Dense metrics come from the best checkpoint by the existing `route_iou + unseen_recall - false_rate` criterion in each run's `metrics.json`. R5 has 30 epochs, a best checkpoint and an ONNX export. Local latency is batch-one OpenCV DNN inference with four threads, including input binding and forward, over 100 validation patches after 10 warmups. The local builds and runtime environment differ from the requested lab container.

## Frontier metric

`eval_frontier.py` evaluates the full 7,880-patch validation set. It mirrors the robot's 5×5 unseen-neighborhood test and takes the farthest-forward pixel in each 8-connected frontier component, with x > 0.2 m. It supplies the same nearby seen-line points to `Restorer.extend_ridge` and uses its 0.6 threshold and 0.4 m length cap.

Precision counts distinct returned points on never-seen ground within 2 cm of the labeled route. Recall counts labeled unseen route pixels within a 0.4 m Euclidean radius of any seed that lie within 2 cm of a returned unseen point. R3 returned 67,478 distinct unseen points, of which 54,021 were within tolerance. It covered 213,946 of 432,669 eligible route pixels. Counts are pooled over the set, and overlapping chains contribute each returned pixel once per patch. The radius can include multiple arms of a loop, and the validation archive contains raster masks without branch order. Branch identity at overlapping crossings and closed-loop robot performance require separate evaluation.

## Training configuration

Both planned variants fine-tune R3 at width 24 on the same 160,000 examples, keeping the batch size, seed and learning rate fixed. The first boosts unseen route positives near the frontiers. The second additionally weights nearby unseen off-route pixels by the square of their detached predicted probability, emphasizing confident false positives. The training weight uses a 32-cell square radius around each candidate frontier pixel for batched GPU pooling. The evaluator uses Euclidean distances and the robot's component seeds.

Run from the repository root in an environment with CUDA and write access to the run directory.

```bash
export OMP_NUM_THREADS=4
py=/home/rvlab/miniforge3/envs/lf_b_dl/bin/python
data=/home/rvlab/restore_data
train=experiments/restore/train.py
common=(--train "$data/train.npz,$data/synth60k.npz,$data/synth60k_b.npz"
        --val "$data/val.npz" --width 24 --batch 64 --epochs 12 --lr 0.0004
        --seed 0 --amp --init "$data/runs/R3_w24_mix/best.pt" --frontier-boost 3)
nice -n 5 "$py" "$train" "${common[@]}" --out "$data/runs/R6_w24_frontier"
nice -n 5 "$py" "$train" "${common[@]}" --out "$data/runs/R6_w24_frontier_neg" --frontier-negative 8
```

The exporter creates a dynamic-batch ONNX with input `x` and output `logit` of shape `[batch, 1, 128, 128]`. A temporary R3 checkpoint export passed ONNX Runtime inference at batch two and matched local OpenCV DNN 4.13 at batch one.

Evaluate each exported ONNX with `eval_frontier.py --model <path> --val /home/rvlab/restore_data/val.npz`. The same script with `--latency` measures OpenCV DNN in the lab when run under `sg docker -c "docker exec line_follower_lab ..."`. Use the repo path under `/ws` and make the validation archive available inside the container. Prefer a candidate with higher chain precision than R3 and comparable chain recall and CPU latency. Keep the current R3 deployment pending R6 evaluation. Model promotion requires measured R6 results.
