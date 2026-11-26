#!/usr/bin/env bash
#
# Convert every experiments/jetson/models/*.onnx to a TensorRT FP16 engine with
# trtexec, using the fixed deployment shapes:
#
#   restore_*.onnx      1x2x128x128
#   seg_depth_*.onnx    1x3x240x320
#
# The shape is baked into the engine as a single optimization profile
# (min == opt == max).  A static ONNX shape is used as-is; a dynamic ONNX input
# gets the fixed shape from the filename rule.  trtexec timing output and the
# TensorRT timing/profile JSON are written under trt_logs/ (or $LOG_DIR).
#
# Run inside the L4T Jazzy container (TensorRT is at /usr/src/tensorrt/bin):
#   /ws/experiments/jetson/to_trt.sh
#
# Optional environment:
#   MODELS_DIR=...       default: <script dir>/models
#   OUT_DIR=...          default: <script dir>/engines
#   LOG_DIR=...          default: <script dir>/trt_logs
#   SEG_SHAPE=...        default: 1x3x240x320
#   RESTORE_SHAPE=...    default: 1x2x128x128
#   TRT_WORKSPACE=...    default: 1024MiB
#   TRTEXEC=...          explicit path to trtexec
#   PYTHON=...           default: python3 (needs onnx for input-name detection)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODELS_DIR="${MODELS_DIR:-$HERE/models}"
OUT_DIR="${OUT_DIR:-$HERE/engines}"
LOG_DIR="${LOG_DIR:-$HERE/trt_logs}"
SEG_SHAPE="${SEG_SHAPE:-1x3x240x320}"
RESTORE_SHAPE="${RESTORE_SHAPE:-1x2x128x128}"
TRT_WORKSPACE="${TRT_WORKSPACE:-1024MiB}"
PYTHON="${PYTHON:-python3}"
SUMMARY="$LOG_DIR/timings.txt"

mkdir -p "$OUT_DIR" "$LOG_DIR"

find_trtexec() {
    if [ -n "${TRTEXEC:-}" ]; then
        printf '%s\n' "$TRTEXEC"
        return 0
    fi
    if command -v trtexec >/dev/null 2>&1; then
        command -v trtexec
        return 0
    fi
    local p
    for p in /usr/src/tensorrt/bin/trtexec /usr/local/bin/trtexec /usr/bin/trtexec; do
        if [ -x "$p" ]; then
            printf '%s\n' "$p"
            return 0
        fi
    done
    return 1
}

if ! TRTEXEC_BIN="$(find_trtexec)"; then
    echo "ERROR: trtexec not found." >&2
    echo "  Put it on PATH, set TRTEXEC=/path/to/trtexec, or use the L4T Jazzy image" >&2
    echo "  (TensorRT tarball usually lives at /usr/src/tensorrt/bin/trtexec)." >&2
    exit 2
fi

echo "trtexec : $TRTEXEC_BIN"
"$TRTEXEC_BIN" --version 2>&1 | head -3 || true
echo "models  : $MODELS_DIR"
echo "engines : $OUT_DIR"
echo "logs    : $LOG_DIR"

shopt -s nullglob
models=("$MODELS_DIR"/*.onnx)
if [ "${#models[@]}" -eq 0 ]; then
    echo "ERROR: no .onnx files in $MODELS_DIR" >&2
    exit 1
fi

{
    echo "# TensorRT FP16 engine conversion"
    echo "# host: $(hostname 2>/dev/null || uname -n)   date: $(date -Is)"
    echo "# trtexec: $TRTEXEC_BIN"
    "$TRTEXEC_BIN" --version 2>&1 | head -3 || true
    printf '\n%-34s %-16s %-8s %s\n' "model" "shape" "status" "engine"
} > "$SUMMARY"

rc=0
for onnx in "${models[@]}"; do
    base="$(basename "$onnx" .onnx)"
    case "$base" in
        restore*)            fallback="$RESTORE_SHAPE" ;;
        seg_depth*|seg*)     fallback="$SEG_SHAPE" ;;
        *)
            fallback="$SEG_SHAPE"
            echo "WARN: $base has no known shape prefix; using $fallback" >&2
            ;;
    esac

    # Print: input_name shape static|dynamic
    info="$("$PYTHON" - "$onnx" "$fallback" <<'PY'
import sys

path, fallback = sys.argv[1], sys.argv[2]
name, shape, kind = "input", fallback, "dynamic"
try:
    import onnx
    model = onnx.load(path, load_external_data=False)
    inp = model.graph.input[0]
    name = inp.name
    dims = []
    dynamic = False
    for d in inp.type.tensor_type.shape.dim:
        if d.dim_value > 0:
            dims.append(int(d.dim_value))
        else:
            dynamic = True
            dims.append(1)
    if len(dims) == 4:
        if dynamic:
            # A dynamic axis is replaced by the deployment shape from the filename
            # rule, so min == opt == max is exactly 1x3x240x320 / 1x2x128x128.
            shape = fallback
            kind = "dynamic"
        else:
            shape = "x".join(str(v) for v in dims)
            kind = "static"
except Exception as exc:  # onnx not installed or unreadable
    print(f"WARN: cannot read ONNX input of {path}: {exc}; assuming {name}:{shape}", file=sys.stderr)
print(name, shape, kind)
PY
)"
    read -r input_name onnx_shape shape_kind <<< "$info"

    engine="$OUT_DIR/${base}_fp16_${onnx_shape}.engine"
    log="$LOG_DIR/${base}_fp16.log"
    times_json="$LOG_DIR/${base}_fp16_times.json"
    profile_json="$LOG_DIR/${base}_fp16_profile.json"

    echo
    echo "== $base  input=$input_name  shape=$onnx_shape ($shape_kind)"

    shape_args=()
    if [ "$shape_kind" = "dynamic" ]; then
        shape_args=(
            "--minShapes=${input_name}:${onnx_shape}"
            "--optShapes=${input_name}:${onnx_shape}"
            "--maxShapes=${input_name}:${onnx_shape}"
        )
    fi

    set +e
    "$TRTEXEC_BIN" \
        --onnx="$onnx" \
        --saveEngine="$engine" \
        --fp16 \
        "${shape_args[@]}" \
        --memPoolSize="workspace:${TRT_WORKSPACE}" \
        --warmUp=500 \
        --duration=10 \
        --avgRuns=100 \
        --exportTimes="$times_json" \
        --exportProfile="$profile_json" \
        --verbose \
        > "$log" 2>&1
    status=$?
    set -e

    if [ "$status" -eq 0 ] && [ -s "$engine" ]; then
        echo "OK   $engine"
        printf '%-34s %-16s %-8s %s\n' "$base" "$onnx_shape" "ok" "$engine" >> "$SUMMARY"
    else
        echo "FAIL $base  (exit $status, see $log)" >&2
        tail -n 30 "$log" >&2 || true
        printf '%-34s %-16s %-8s %s\n' "$base" "$onnx_shape" "FAIL" "$log" >> "$SUMMARY"
        rc=1
    fi

    {
        echo
        echo "## $base  shape=$onnx_shape  engine=$engine"
        grep -E 'Engine built in|GPU Compute Time|Latency:|Throughput:|Total Host Walltime' "$log" || true
    } >> "$SUMMARY"
done

echo
echo "Timing summary : $SUMMARY"
echo "Engines        : $OUT_DIR"
exit "$rc"
