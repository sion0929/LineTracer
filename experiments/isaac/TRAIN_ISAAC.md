# Train on Isaac decoy1000

Run from `/home/rvlab/26_robotprogramming_project` in Bash.

```bash
cd /home/rvlab/26_robotprogramming_project
PY=/home/rvlab/miniforge3/envs/lf_b_dl/bin/python
DATA=/home/rvlab/isaac_datasets/26_robotprogramming/decoy1000
INIT=/home/rvlab/orca/workspaces/26_robotprogramming_project/b-dlseg/experiments/dl/runs/round3-v4/round4-lead/T4_small/best.pt
```

## Near segmentation and depth

```bash
OMP_NUM_THREADS=4 nice -n 10 "$PY" experiments/isaac/train_near.py \
  --device cuda --data "$DATA" --small-model --init "$INIT" \
  --epochs 10 --batch 12 --workers 2 \
  --out experiments/isaac/runs/decoy1000-near-v4-small
```

`T4_small` had the highest recorded joint score among the completed 320×240 small V4 runs with a learned depth head, positive depth loss, and joint checkpoint selection. Its source report records epoch 39, joint score 0.866725, line IoU 0.884432, and near line depth MAE 1.770639 cm. These are source-run results on its own validation data. Omit `--init "$INIT"` for a scratch run if the checkpoint is unavailable on the training host.

Both splits are indexed once at launch and use completed maps marked by `summary.json`. Wait for the required maps before launching a full run. Repeat `--data /path/to/older-dataset` after the main `--data` to append older training frames. Each extra accepts a dataset root with a `train/` directory or a directory containing map folders directly. Supported map contents are Isaac NPZ files listed in `frames.jsonl` or Gazebo `image/`, `mask/`, and optional millimetre `depth/` PNGs. Mask value 1 means line. A Gazebo map without a depth directory contributes segmentation supervision. Validation always uses the main dataset's `val/`.

The output directory must be new. It receives `epochs.jsonl`, `best.pt`, and `line_seg_depth_v4.onnx`. Select best by validation line IoU minus 0.01 times near line MAE in cm over labeled line depths of 0.20–0.95 m. The ONNX consumes float32 RGB divided by 255 as `rgb` with shape `1×3×240×320`. It returns `logits` with shape `1×2×240×320` and camera-axis `depth_m` with shape `1×1×240×320`.

## Far-band segmentation

Convert the completed maps. Re-run the command after collection finishes to include newly completed maps.

```bash
OMP_NUM_THREADS=4 nice -n 10 "$PY" experiments/isaac/make_farband_from_isaac.py \
  --data "$DATA" --split all
```

The trainer holds out the last unique filename stems across every input folder. The converted `z_decoy_0000`–`z_decoy_0949` stems sort after the existing `tile_*` stems. With all 50 Isaac validation maps present, `--val-maps 50` holds out exactly `z_decoy_0900`–`z_decoy_0949`. Older Gazebo strips are therefore training inputs for this mixed run.

Run the following block after conversion. The guard checks the archive inventory and the trainer's actual filename ordering before it starts training.

```bash
OMP_NUM_THREADS=4 nice -n 10 "$PY" - <<'PY' &&
from pathlib import Path

root = Path("experiments/farband")
isaac = {p.stem for p in (root / "data_isaac").glob("*.npz")}
expected = {f"z_decoy_{seed:04d}" for seed in range(950)}
held_out = {f"z_decoy_{seed:04d}" for seed in range(900, 950)}
legacy = {p.stem for folder in ("data", "data_hill") for p in (root / folder).glob("*.npz")}
assert isaac == expected, f"Isaac archives missing={sorted(expected - isaac)}, extra={sorted(isaac - expected)}"
assert not (isaac & legacy) and set(sorted(isaac | legacy)[-50:]) == held_out
PY
OMP_NUM_THREADS=4 nice -n 10 "$PY" experiments/farband/train.py \
  --data experiments/farband/data,experiments/farband/data_hill,experiments/farband/data_isaac \
  --gpu-aug --val-maps 50 --epochs 40 --batch 48 --width 16 --hscale 1 \
  --out experiments/farband/runs/decoy1000-mixed
```

The trainer loads all strips into host memory and `--gpu-aug` keeps the training strips on GPU. At 40 frames per map, 900 Isaac maps alone require about 5.49 GiB of GPU storage for the uint8 inputs and targets at `--hscale 1`. Budget additional memory for legacy strips and training tensors. The CLI also supports `--hscale 2` to halve the horizontal resolution and strip storage if needed. Reducing batch size reduces activation memory.

## CPU smoke run

The following commands ran with 4 completed validation maps and 160 frames. Choose a fresh output directory when repeating near training.

```bash
PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=4 nice -n 10 "$PY" \
  experiments/isaac/train_near.py --device cpu --small-model --init "$INIT" \
  --train-split val --epochs 1 --batch 2 --workers 0 --max-train-steps 3 \
  --out experiments/isaac/near_cpu_smoke
PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=4 nice -n 10 "$PY" \
  experiments/isaac/make_farband_from_isaac.py --split val
```

Near validation reused the training split for this smoke run. OpenCV 4.13.0 loaded the exported ONNX and forwarded a real resized validation RGB frame. It returned finite tensors of shapes `(1, 2, 240, 320)` and `(1, 1, 240, 320)`. The existing far-band loader read all four archives, and every stored RGB strip and binary target matched its source crop.
