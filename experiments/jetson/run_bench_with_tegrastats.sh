#!/usr/bin/env bash
#
# Run bench_models.py while sampling tegrastats every 200 ms, then summarise
# GPU %, CPU %, RAM, temperatures and power.
#
#   /ws/experiments/jetson/run_bench_with_tegrastats.sh --runs 200 --threads 4
#
# Outputs (default: experiments/results/lead/ds/w2/):
#   bench_<host>.json
#   tegrastats_<host>_<stamp>.log
#   tegrastats_<host>_<stamp>_summary.json
#   tegrastats_<host>_<stamp>_summary.txt
#   env_<host>_<stamp>.txt          (nvpmodel, jetson_clocks, kernel, L4T)
#
# tegrastats is part of the host L4T BSP, not the ROS image.  If it is not in the
# container, mount it and point the script at it:
#   docker run ... -v /usr/bin/tegrastats:/host/usr/bin/tegrastats:ro \
#     -e TEGRASTATS=/host/usr/bin/tegrastats ...
#
# Optional environment:
#   OUT_DIR=...             default: <script dir>/../results/lead/ds/w2
#   TEGRA_INTERVAL_MS=...   default: 200
#   PYTHON=...              default: python3
#   TEGRASTATS=...          explicit path to tegrastats
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${OUT_DIR:-$HERE/../results/lead/ds/w2}"
INTERVAL_MS="${TEGRA_INTERVAL_MS:-200}"
PYTHON="${PYTHON:-python3}"

HOST="$(hostname 2>/dev/null || uname -n)"
STAMP="$(date +%Y%m%dT%H%M%S)"
TEGRA_LOG="$OUT_DIR/tegrastats_${HOST}_${STAMP}.log"
SUMMARY_JSON="$OUT_DIR/tegrastats_${HOST}_${STAMP}_summary.json"
SUMMARY_TXT="$OUT_DIR/tegrastats_${HOST}_${STAMP}_summary.txt"
BENCH_JSON="$OUT_DIR/bench_${HOST}.json"
ENV_LOG="$OUT_DIR/env_${HOST}_${STAMP}.txt"

mkdir -p "$OUT_DIR"

find_tegrastats() {
    if [ -n "${TEGRASTATS:-}" ]; then
        printf '%s\n' "$TEGRASTATS"
        return 0
    fi
    if command -v tegrastats >/dev/null 2>&1; then
        command -v tegrastats
        return 0
    fi
    local p
    for p in /usr/bin/tegrastats /usr/local/bin/tegrastats /host/usr/bin/tegrastats; do
        if [ -x "$p" ]; then
            printf '%s\n' "$p"
            return 0
        fi
    done
    return 1
}

if ! TEGRASTATS_BIN="$(find_tegrastats)"; then
    echo "ERROR: tegrastats not found in this container." >&2
    echo "  Mount it from the host: -v /usr/bin/tegrastats:/usr/bin/tegrastats:ro" >&2
    echo "  or set TEGRASTATS=/host/usr/bin/tegrastats (with the same bind mount)." >&2
    exit 2
fi
if [ ! -x "$TEGRASTATS_BIN" ]; then
    echo "ERROR: tegrastats is not executable: $TEGRASTATS_BIN" >&2
    exit 2
fi

{
    echo "host: $HOST"
    echo "date: $(date -Is)"
    echo "tegrastats: $TEGRASTATS_BIN"
    echo "interval_ms: $INTERVAL_MS"
    echo
    "$TEGRASTATS_BIN" --version 2>&1 | head -3 || true
    echo
    nvpmodel -q 2>&1 || true
    echo
    jetson_clocks --show 2>&1 || true
    echo
    uname -a
    echo
    cat /etc/nv_tegra_release 2>/dev/null || true
} > "$ENV_LOG" 2>&1 || true

echo "tegrastats : $TEGRASTATS_BIN --interval $INTERVAL_MS"
echo "log        : $TEGRA_LOG"
echo "bench json : $BENCH_JSON"
echo "env        : $ENV_LOG"

TEGRA_PID=""
cleanup() {
    if [ -n "${TEGRA_PID:-}" ] && kill -0 "$TEGRA_PID" 2>/dev/null; then
        kill -INT "$TEGRA_PID" 2>/dev/null || true
        for _ in {1..20}; do
            kill -0 "$TEGRA_PID" 2>/dev/null || break
            sleep 0.1
        done
        kill -TERM "$TEGRA_PID" 2>/dev/null || true
        wait "$TEGRA_PID" 2>/dev/null || true
    fi
    TEGRA_PID=""
}
trap cleanup EXIT INT TERM

"$TEGRASTATS_BIN" --interval "$INTERVAL_MS" > "$TEGRA_LOG" 2>&1 &
TEGRA_PID=$!
sleep 0.3
if ! kill -0 "$TEGRA_PID" 2>/dev/null; then
    echo "ERROR: tegrastats exited immediately; see $TEGRA_LOG" >&2
    cat "$TEGRA_LOG" >&2 || true
    exit 2
fi

bench_rc=0
"$PYTHON" "$HERE/bench_models.py" --out "$BENCH_JSON" "$@" || bench_rc=$?

cleanup
trap - EXIT INT TERM

"$PYTHON" - "$TEGRA_LOG" "$SUMMARY_JSON" "$SUMMARY_TXT" "$BENCH_JSON" "$INTERVAL_MS" "$HOST" <<'PY'
import json
import re
import statistics
import sys
from pathlib import Path

log_path, out_json, out_txt, bench_path, interval_ms, host = sys.argv[1:7]
interval_ms = int(interval_ms)
lines = Path(log_path).read_text(errors="replace").splitlines()

samples = []
for line in lines:
    # A real tegrastats line always has RAM; skip banners and empty lines.
    if "RAM" not in line:
        continue
    s = {}

    m = re.search(r"RAM\s+(\d+)/(\d+)MB", line)
    if m:
        s["ram_used_mb"] = int(m.group(1))
        s["ram_total_mb"] = int(m.group(2))

    m = re.search(r"CPU\s+\[([^\]]+)\]", line)
    if m:
        cores = [int(x) for x in re.findall(r"(\d+)%", m.group(1))]
        if cores:
            s["cpu_pct"] = sum(cores) / len(cores)
            s["cpu_core_max_pct"] = max(cores)

    m = re.search(r"GR3D_FREQ\s+(\d+)%", line)
    if m:
        s["gpu_pct"] = int(m.group(1))

    for m in re.finditer(r"(CPU|GPU|SOC\d*|tj)@([\d.]+)C", line):
        key = {"CPU": "cpu_temp_c", "GPU": "gpu_temp_c", "tj": "tj_temp_c"}.get(
            m.group(1), "soc_temp_c")
        s[key] = max(s.get(key, float("-inf")), float(m.group(2)))

    for key, pat in (
        ("vdd_in_mw", r"VDD_IN\s+(\d+)mW"),
        ("vdd_cpu_gpu_cv_mw", r"VDD_CPU_GPU_CV\s+(\d+)mW"),
        ("pom_5v_in_mw", r"POM_5V_IN\s+(\d+)(?:mW)?"),
    ):
        m = re.search(pat, line)
        if m:
            s[key] = max(s.get(key, 0), int(m.group(1)))

    if s:
        samples.append(s)


def values(key):
    return [s[key] for s in samples if key in s]


def stats(key):
    vals = values(key)
    if not vals:
        return None
    return {
        "mean": round(statistics.fmean(vals), 2),
        "max": round(max(vals), 2),
        "min": round(min(vals), 2),
        "n": len(vals),
    }


sample_count = len(samples)
duration_s = round(sample_count * interval_ms / 1000.0, 2)

gpu = stats("gpu_pct")
cpu = stats("cpu_pct")
cpu_core = stats("cpu_core_max_pct")
ram = stats("ram_used_mb")
ram_total = max(values("ram_total_mb"), default=None)

temp = {
    "cpu_max_c": max(values("cpu_temp_c"), default=None),
    "gpu_max_c": max(values("gpu_temp_c"), default=None),
    "soc_max_c": max(values("soc_temp_c"), default=None),
    "tj_max_c": max(values("tj_temp_c"), default=None),
}

power = {}
for label, key in (("vdd_in", "vdd_in_mw"),
                   ("vdd_cpu_gpu_cv", "vdd_cpu_gpu_cv_mw"),
                   ("pom_5v_in", "pom_5v_in_mw")):
    vals = [v / 1000.0 for v in values(key)]
    if vals:
        power = {
            "rail": label,
            "mean_w": round(statistics.fmean(vals), 3),
            "max_w": round(max(vals), 3),
            "energy_j": round(statistics.fmean(vals) * duration_s, 2),
        }
        break

bench = []
try:
    b = json.loads(Path(bench_path).read_text())
    for row in b.get("results", []):
        bench.append({k: row.get(k) for k in
                      ("model", "backend", "input", "median_ms", "p95_ms", "max_ms", "error")})
except Exception as exc:
    bench = [{"error": f"could not read {bench_path}: {exc}"}]

summary = {
    "host": host,
    "tegrastats_log": log_path,
    "bench_json": bench_path,
    "interval_ms": interval_ms,
    "samples": sample_count,
    "duration_s": duration_s,
    "gpu_pct": gpu,
    "cpu_pct": cpu,
    "cpu_core_max_pct": cpu_core,
    "ram_used_mb": ram,
    "ram_total_mb": ram_total,
    "temperature_c": temp,
    "power": power,
    "bench": bench,
}
Path(out_json).write_text(json.dumps(summary, indent=2) + "\n")

def fmt(v, suffix="", nd=1):
    return "n/a" if v is None else f"{v:.{nd}f}{suffix}"

lines_out = [
    "================ tegrastats summary ================",
    f"host          : {host}",
    f"tegrastats    : {log_path}",
    f"samples       : {sample_count} over {duration_s:.1f} s ({interval_ms} ms)",
    f"GPU           : mean {fmt(gpu and gpu['mean'], '%')}  max {fmt(gpu and gpu['max'], '%')}",
    f"CPU           : mean {fmt(cpu and cpu['mean'], '%')}  max-core {fmt(cpu_core and cpu_core['max'], '%')}",
    f"RAM           : mean {fmt(ram and ram['mean'], ' MB', 0)} / {fmt(ram_total, ' MB', 0)}"
    f"  max {fmt(ram and ram['max'], ' MB', 0)}",
    f"temperature   : CPU max {fmt(temp['cpu_max_c'], ' C')}  GPU max {fmt(temp['gpu_max_c'], ' C')}"
    f"  SOC max {fmt(temp['soc_max_c'], ' C')}  tj max {fmt(temp['tj_max_c'], ' C')}",
]
if power:
    lines_out.append(
        f"power         : {power['rail']} mean {power['mean_w']:.2f} W"
        f"  max {power['max_w']:.2f} W  energy {power['energy_j']:.1f} J")
else:
    lines_out.append("power         : n/a (no VDD_IN / VDD_CPU_GPU_CV / POM_5V_IN field)")
lines_out += [
    f"bench json    : {bench_path}",
    "----------------------------------------------------",
]
for row in bench:
    if "error" in row and row.get("error"):
        lines_out.append(f"bench error   : {row['error']}")
        continue
    lines_out.append(
        f"{row.get('model', '?'):28s} {row.get('backend', '?'):30s}"
        f" median {row.get('median_ms', 'n/a')} ms  p95 {row.get('p95_ms', 'n/a')} ms")
lines_out.append("====================================================")

text = "\n".join(lines_out) + "\n"
Path(out_txt).write_text(text)
print(text, end="")
PY

echo "summary json : $SUMMARY_JSON"
echo "summary txt  : $SUMMARY_TXT"

exit "$bench_rc"
