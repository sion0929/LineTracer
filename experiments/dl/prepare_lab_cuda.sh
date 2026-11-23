#!/usr/bin/env bash
# Stage the CUDA runtime in /ws without changing the lab image or other environments.
set -euo pipefail
host_python=${1:-/home/rvlab/miniforge3/envs/lf_b_dl/bin/python}
nvidia_root=${2:-/home/rvlab/miniforge3/envs/lerobot/lib/python3.12/site-packages/nvidia}
runtime=experiments/dl/runs/lab-runtime-cuda
libraries=experiments/dl/runs/lab-cuda-libs
if [[ ! -f "$runtime/onnxruntime/capi/libonnxruntime_providers_cuda.so" ]]; then
  "$host_python" -m pip --isolated install --index-url https://pypi.org/simple --no-deps --no-cache-dir \
    --target "$runtime" onnxruntime-gpu==1.32.0 flatbuffers packaging
fi
mkdir -p "$libraries"
stage() {
  local src=$1 dest
  dest="$libraries/${src##*/}"
  [[ -f "$src" ]] || { echo "Missing CUDA library: $src" >&2; return 1; }
  [[ -e "$dest" ]] || ln "$src" "$dest" || cp -p "$src" "$dest"
}
for library in libcublasLt.so.13 libcublas.so.13 libcurand.so.10 libcudart.so.13; do
  stage "$nvidia_root/cu13/lib/$library"
done
for library in "$nvidia_root"/cudnn/lib/libcudnn*.so.9 "$nvidia_root"/cu13/lib/libnvrtc*.so*; do
  stage "$library"
done
ln -sfnT libcudnn.so.9 "$libraries/libcudnn.so"
