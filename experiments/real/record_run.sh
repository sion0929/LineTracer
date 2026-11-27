#!/usr/bin/env bash
# Record one real-robot run with rosbag2 and write a metadata YAML.
#
# Usage:
#   experiments/real/record_run.sh RUN_NAME [options]
#
# Options:
#   --notes TEXT       free-form note for the metadata (default: empty)
#   --robot NAME       robot name (default: $ROBOT or turtlebot3_waffle_pi)
#   --battery VALUE    battery state, e.g. "11.9 V, 3S, 90%" (default: $BATTERY or unknown)
#   --out DIR          output directory (default: $OUT_DIR or <repo>/bags/real)
#   --duration SEC     stop after SEC seconds (SIGINT, clean for rosbag2)
#   --camera-mode M    auto | raw | compressed (default: auto)
#   --dry-run          print the command and metadata, record nothing
#   -h, --help         show this help
#
# Topics: camera (raw or compressed), camera_info, odom, imu, cmd_vel, tf,
#         tf_static, line_follower/debug. Missing topics are skipped.
# mcap is used when its rosbag2 plugin is available; otherwise sqlite3.
set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo=$(cd "$script_dir/../.." && pwd)

usage() {
    cat <<'EOF'
Usage:
  experiments/real/record_run.sh RUN_NAME [options]

Options:
  --notes TEXT       free-form note for the metadata (default: empty)
  --robot NAME       robot name (default: $ROBOT or turtlebot3_waffle_pi)
  --battery VALUE    battery state, e.g. "11.9 V, 3S, 90%" (default: $BATTERY or unknown)
  --out DIR          output directory (default: $OUT_DIR or <repo>/bags/real)
  --duration SEC     stop after SEC seconds (SIGINT, clean for rosbag2)
  --camera-mode M    auto | raw | compressed (default: auto)
  --dry-run          print the command and metadata, record nothing
  -h, --help         show this help

Topics: camera (raw or compressed), camera_info, odom, imu, cmd_vel, tf,
        tf_static, line_follower/debug. Missing topics are skipped.
mcap is used when its rosbag2 plugin is available; otherwise sqlite3.
EOF
}

name=""
notes=""
robot="${ROBOT:-turtlebot3_waffle_pi}"
battery="${BATTERY:-unknown}"
out="${OUT_DIR:-$repo/bags/real}"
duration=""
camera_mode="${CAMERA_MODE:-auto}"
dry_run=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --notes) notes="${2:-}"; shift 2 ;;
        --robot) robot="${2:-}"; shift 2 ;;
        --battery) battery="${2:-}"; shift 2 ;;
        --out) out="${2:-}"; shift 2 ;;
        --duration) duration="${2:-}"; shift 2 ;;
        --camera-mode) camera_mode="${2:-}"; shift 2 ;;
        --dry-run) dry_run=1; shift ;;
        -h|--help) usage; exit 0 ;;
        -*) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
        *)
            if [[ -z "$name" ]]; then name="$1"; shift
            else echo "unexpected argument: $1" >&2; usage >&2; exit 2; fi
            ;;
    esac
done

if [[ -z "$name" ]]; then
    echo "run name is required" >&2
    usage >&2
    exit 2
fi

case "$camera_mode" in
    auto|raw|compressed) ;;
    *) echo "--camera-mode must be auto, raw or compressed" >&2; exit 2 ;;
esac

if ! [[ "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]] && [[ -n "$duration" ]]; then
    echo "--duration must be a number of seconds" >&2
    exit 2
fi

# Keep names portable (ASCII letters, digits, dot, dash, underscore).
safe_name=$(printf '%s' "$name" | tr -c 'A-Za-z0-9_.-' '_')
stamp=$(date +%Y%m%d_%H%M%S)
iso_date=$(date -Iseconds)
run_id="${stamp}_${safe_name}"
bag="$out/$run_id"
meta="$out/$run_id.yaml"

if [[ -e "$bag" ]]; then
    echo "already exists: $bag" >&2
    exit 1
fi

mkdir -p "$out"

commit="unknown"
if command -v git >/dev/null 2>&1; then
    commit=$(git -C "$repo" rev-parse --short HEAD 2>/dev/null || true)
    [[ -n "$commit" ]] || commit="unknown"
fi

if ! command -v python3 >/dev/null 2>&1; then
    echo "python3 is required to write the metadata YAML" >&2
    exit 1
fi

ros2_ok=0
if command -v ros2 >/dev/null 2>&1; then ros2_ok=1; fi
if [[ $ros2_ok -eq 0 && $dry_run -eq 0 ]]; then
    echo "ros2 not found. Source the ROS 2 environment first." >&2
    exit 1
fi

available=""
if [[ $ros2_ok -eq 1 ]]; then
    available=$(ros2 topic list 2>/dev/null || true)
fi

has_topic() { [[ -n "$available" ]] && grep -Fxq "$1" <<<"$available"; }

raw_cam=/camera/image_raw
comp_cam=/camera/image_raw/compressed
case "$camera_mode" in
    raw) cam=$raw_cam ;;
    compressed) cam=$comp_cam ;;
    auto)
        if [[ -n "$available" ]] && has_topic "$raw_cam"; then cam=$raw_cam
        elif [[ -n "$available" ]] && has_topic "$comp_cam"; then cam=$comp_cam
        else cam=$raw_cam; fi
        ;;
esac

wanted=(
    "$cam"
    /camera/camera_info
    /odom
    /imu
    /cmd_vel
    /tf
    /tf_static
    /line_follower/debug
)
topics=()
for topic in "${wanted[@]}"; do
    if [[ -z "$available" ]] || has_topic "$topic"; then
        topics+=("$topic")
    else
        echo "skip (not present): $topic"
    fi
done

if [[ ${#topics[@]} -eq 0 ]]; then
    echo "no topics to record" >&2
    exit 1
fi

storage=()
storage_id=sqlite3
qos_args=()
qos_file=""
if [[ $ros2_ok -eq 1 ]]; then
    bag_help=$(ros2 bag record --help 2>/dev/null || true)
    if ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1 || grep -q 'mcap' <<<"$bag_help"; then
        storage=(-s mcap)
        storage_id=mcap
        if grep -q 'storage-preset-profile' <<<"$bag_help"; then
            storage+=(--storage-preset-profile zstd_fast)
        fi
    fi
    # /tf_static is published once with transient_local QoS; a plain subscriber
    # may start too late to receive it.
    if grep -q 'qos-profile-overrides-path' <<<"$bag_help"; then
        qos_file="$out/.qos_${run_id}.yaml"
        cat >"$qos_file" <<'YAML'
/tf_static:
  durability: transient_local
  history: keep_all
  depth: 1000
YAML
        qos_args=(--qos-profile-overrides-path "$qos_file")
    fi
fi

write_meta() {
    local run_duration="${1:-}"
    local bag_size="${2:-}"
    python3 - "$meta" "$run_id" "$name" "$iso_date" "$robot" "$battery" \
        "$notes" "$commit" "$cam" "$storage_id" "$run_duration" "$bag_size" \
        "$bag" "$repo" "${topics[@]}" <<'PY'
import json
import os
import socket
import sys

(meta, run_id, name, iso_date, robot, battery, notes, commit, cam,
 storage_id, run_duration, bag_size, bag, repo, *topics) = sys.argv[1:]

def quoted(value):
    return json.dumps(value, ensure_ascii=False)

def number(value):
    if value in ('', None):
        return 'null'
    try:
        float(value)
        return value
    except ValueError:
        return quoted(value)

lines = [
    '# Written by experiments/real/record_run.sh',
    f'run: {quoted(run_id)}',
    f'name: {quoted(name)}',
    f'date: {quoted(iso_date)}',
    f'robot: {quoted(robot)}',
    f'host: {quoted(socket.gethostname())}',
    f'battery: {quoted(battery)}',
    f'notes: {quoted(notes)}',
    f'git_commit: {quoted(commit)}',
    f'repo: {quoted(repo)}',
    f'camera_topic: {quoted(cam)}',
    f'storage: {quoted(storage_id)}',
    f'ros_domain_id: {quoted(os.environ.get("ROS_DOMAIN_ID", "0"))}',
    f'duration_sec: {number(run_duration)}',
    f'bag_size_bytes: {number(bag_size)}',
    f'bag_path: {quoted(bag)}',
    'topics:',
]
lines += [f'  - {quoted(topic)}' for topic in topics]
with open(meta, 'w', encoding='utf-8') as stream:
    stream.write('\n'.join(lines) + '\n')
PY
}

write_meta "" ""

cmd=(ros2 bag record "${storage[@]}" "${qos_args[@]}" -o "$bag" "${topics[@]}")
if [[ -n "$duration" ]]; then
    cmd=(timeout --foreground -s INT "$duration" "${cmd[@]}")
fi

echo "bag:   $bag"
echo "meta:  $meta"
echo "topics: ${topics[*]}"
echo "cmd:   ${cmd[*]}"

if [[ $dry_run -eq 1 ]]; then
    echo
    echo "--- metadata (dry run) ---"
    cat "$meta"
    rm -f "$meta"
    if [[ -n "$qos_file" ]]; then rm -f "$qos_file"; fi
    exit 0
fi

# Ctrl-C is delivered to rosbag2 too. The trap keeps this shell alive so the
# metadata below is still updated after the recorder stops.
trap ': ' INT
start_epoch=$(date +%s)
set +e
"${cmd[@]}"
rc=$?
set -e
trap - INT
end_epoch=$(date +%s)
if [[ -n "$qos_file" ]]; then rm -f "$qos_file"; fi

actual_duration=$((end_epoch - start_epoch))
bag_size=""
if [[ -d "$bag" ]]; then
    bag_size=$(du -sb "$bag" 2>/dev/null | cut -f1 || true)
fi
write_meta "$actual_duration" "$bag_size"

echo
echo "saved bag:  $bag"
echo "saved meta: $meta"
if [[ -d "$bag" ]] && command -v ros2 >/dev/null 2>&1; then
    ros2 bag info "$bag" || true
fi

if [[ $rc -ne 0 && $rc -ne 124 && $rc -ne 130 ]]; then
    echo "rosbag2 exited with status $rc" >&2
    exit "$rc"
fi
