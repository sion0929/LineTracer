#!/usr/bin/env bash
# Replay each route in a fresh Isaac process, then score the resulting maps.
set -euo pipefail

if [[ ${1:-} == --help || ${1:-} == -h || $# != 3 ]]; then
    echo "Usage: run_replay_eval.sh MAPLIST OUTDIR PROFILE_OVERLAY.yaml"
    echo "MAPLIST: one 'official' or JSON path per line; relative paths use the list directory, then repo root."
    echo "ISAAC_REPLAY_SPEED defaults to 0.2 m/s; ISAAC_MAX_SIM_SECONDS defaults to 300."
    [[ ${1:-} == --help || ${1:-} == -h ]]
    exit
fi

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
maplist=$(realpath -e -- "$1")
out=$(realpath -m -- "$2")
overlay=$(realpath -e -- "$3")
speed=${ISAAC_REPLAY_SPEED:-0.2}
limit=${ISAAC_MAX_SIM_SECONDS:-300}
[[ -f $maplist && -f $overlay ]] || { echo "MAPLIST and PROFILE_OVERLAY must be files" >&2; exit 2; }

maps=()
while IFS= read -r entry || [[ -n $entry ]]; do
    entry=${entry%$'\r'}
    entry=${entry#"${entry%%[![:space:]]*}"}
    entry=${entry%"${entry##*[![:space:]]}"}
    [[ -n $entry && $entry != \#* ]] || continue
    if [[ $entry != official ]]; then
        if [[ $entry == /* ]]; then
            entry=$(realpath -e -- "$entry")
        elif [[ -f $(dirname "$maplist")/$entry ]]; then
            entry=$(realpath -e -- "$(dirname "$maplist")/$entry")
        else
            entry=$(realpath -e -- "$root/$entry")
        fi
        [[ -f $entry && $entry == *.json ]] || { echo "Expected a JSON map: $entry" >&2; exit 2; }
    fi
    maps+=("$entry")
done <"$maplist"
(( ${#maps[@]} )) || { echo "No maps in $maplist" >&2; exit 2; }

python3 -c 'import numpy, scipy, matplotlib' ||
    { echo "Use a host Python environment with NumPy, SciPy and Matplotlib." >&2; exit 2; }

mkdir -p -- "$out"/{runs,failed}
exec 9>"$out/.replay.lock"
flock -n 9 || { echo "Another replay batch is using $out" >&2; exit 1; }
overlay_hash=$(sha256sum <"$overlay")
manifest=$(printf '%s\n' "$speed" "$limit" "$overlay" "$overlay_hash" \
    "${ISAAC_FOLLOWER_ARGS:-}" "${ISAAC_HOLD:-1}" "${maps[@]}")
if [[ -f $out/batch.txt ]]; then
    [[ $(<"$out/batch.txt") == "$manifest" ]] ||
        { echo "Replay configuration differs from $out/batch.txt. Choose another OUTDIR." >&2; exit 2; }
else
    printf '%s\n' "$manifest" >"$out/batch.txt"
fi

finished() {
    [[ -s $1/job.json && -s $1/map_world.npz && -f $1/result.json ]] || return 1
    python3 - "$1/result.json" <<'PY'
import json
import sys
from pathlib import Path

try:
    result = json.loads(Path(sys.argv[1]).read_text())
except (OSError, ValueError):
    raise SystemExit(1)
raise SystemExit(not (result.get("complete") is True and result.get("reason") == "finished"))
PY
}

archive() {
    local saved
    saved=$(mktemp -d "$out/failed/$name.XXXXXX")
    mv -- "$run" "$saved/$name"
}

failed=0
for index in "${!maps[@]}"; do
    map=${maps[index]}
    label=${map##*/}
    printf -v name '%04d_%s' "$((index + 1))" "${label%.json}"
    run="$out/runs/$name"
    if finished "$run"; then
        echo "Skipping completed $name"
        continue
    fi
    if [[ -e $run ]]; then archive; fi
    echo "Replaying $name"
    if bash "$root/experiments/isaac/run_isaac_drive.sh" --map-dump \
        --map "$map" --out "$run" --profile-overlay "$overlay" \
        --replay-speed "$speed" --max-sim-seconds "$limit" --port "${ISAAC_PORT:-18990}" && finished "$run"; then
        continue
    fi
    echo "Replay incomplete: $name" >&2
    if [[ -e $run ]]; then archive; fi
    failed=1
done

python3 "$root/experiments/eval/perception_map_eval.py" "$out/runs" "$out/eval"
exit "$failed"
