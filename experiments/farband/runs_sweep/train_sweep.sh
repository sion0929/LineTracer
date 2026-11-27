#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."

python=/home/rvlab/miniforge3/envs/lf_b_dl/bin/python
export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 PYTHONDONTWRITEBYTECODE=1

train() {
    local width=$1 epochs=$2
    local out="runs_sweep/w${width}_h2_e${epochs}"
    mkdir -- "$out"
    nice -n 5 "$python" -u -c '
import runpy
import cv2
import torch
torch.set_num_threads(4)
cv2.setNumThreads(1)
torch.manual_seed(0)
runpy.run_path("train.py", run_name="__main__")
' --data data,data_hill --out "$out" --val-maps 10 --epochs "$epochs" \
        --width "$width" --hscale 2 --target 0.2 --lr 0.002 --batch 48 \
        2>&1 | tee "$out/train.log"
}

"$python" -c 'import onnx, torch; assert torch.cuda.is_available(), "CUDA unavailable"'
train 12 14
train 16 14
train 8 30
train 10 14
