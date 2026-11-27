#!/usr/bin/env bash
# Run the renderer on the host and the follower/relay inside the existing lab.
set -euo pipefail

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    cat <<'EOF'
Usage: run_isaac_drive.sh --map official|JSON --out DIR [--run-user|--run-split|--map-dump]
                          [--replay-speed V] [--profile-overlay YAML] [renderer options]
Environment: ISAAC_HOME, LAB_NAME, ROS_DOMAIN_ID (90–109), ISAAC_HOST, ISAAC_BIND,
ISAAC_HOLD (wall periods per sim tick, default 1), ISAAC_FOLLOWER_ARGS (extra ROS args for the follower, e.g. "-p perception:=depth -p depth.model_path:=/ws/...onnx").
Use --run-user for a follower with no Gazebo truth logger.
Use --run-split for the same follower as four nodes (perception, map, planning, control;
docs/architecture/NODE_SPLIT.md). It needs line_follower_interfaces built in /ws/install.
EOF
    exit 0
fi
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
isaac=${ISAAC_HOME:-/home/rvlab/isaacsim}
lab=${LAB_NAME:-line_follower_lab}
domain=${ROS_DOMAIN_ID:-109}
relay_host=${ISAAC_HOST:-172.17.0.1}
bind=${ISAAC_BIND:-$relay_host}
profile=xfar
port=18990
out=
map=
overlay=
drive_args=()
while (( $# )); do
    case $1 in
        --run-user) profile=run_user; shift ;;
        --run-split) profile=run_split; shift ;;
        --map-dump) profile=map_dump; shift ;;
        --map) map=${2:?--map needs official or a JSON path}; shift 2 ;;
        --profile-overlay) overlay=${2:?--profile-overlay needs a YAML path}; shift 2 ;;
        --out) out=${2:?--out needs a directory}; shift 2 ;;
        --port) port=${2:?--port needs a value}; shift 2 ;;
        --bind) bind=${2:?--bind needs an address}; shift 2 ;;
        *) drive_args+=("$1"); shift ;;
    esac
done
[[ -n $out ]] || { echo "--out is required" >&2; exit 2; }
[[ -n $map ]] || { echo "--map is required" >&2; exit 2; }
if [[ -n $overlay ]]; then
    [[ $profile == map_dump && -r $overlay ]] ||
        { echo "--profile-overlay requires --map-dump and a readable YAML file" >&2; exit 2; }
fi
[[ $domain =~ ^(9[0-9]|10[0-9])$ ]] || { echo "ROS_DOMAIN_ID must be 90–109" >&2; exit 2; }
[[ $port =~ ^[0-9]{1,5}$ ]] && (( 10#$port > 0 && 10#$port <= 65535 )) ||
    { echo "Invalid TCP port" >&2; exit 2; }
[[ -x $isaac/python.sh ]] || { echo "Missing $isaac/python.sh" >&2; exit 2; }
out=$(realpath -m -- "$out")
mkdir -p -- "$(dirname "$out")"
mkdir -- "$out" || { echo "Use a fresh output directory: $out" >&2; exit 2; }

# The in-container supervisor treats EOF on docker exec stdin as a stop request.
# The launcher alone holds the write end of the FIFO.
control=$(mktemp -d)
shared=
dump_dir=
host_pid=
lab_pid=
keepalive=
cleanup() {
    local status=$?
    trap - EXIT INT TERM
    if [[ -n $keepalive ]]; then exec {keepalive}>&-; fi
    if [[ -n $host_pid && ! -f $out/result.json ]]; then
        kill -TERM -- "-$host_pid" "$host_pid" 2>/dev/null || true
    fi
    for ((i=0; i<50; i++)); do
        if { [[ -z $lab_pid ]] || ! kill -0 "$lab_pid" 2>/dev/null; } &&
           { [[ -z $host_pid ]] || ! kill -0 "$host_pid" 2>/dev/null; }; then
            break
        fi
        sleep 0.1
    done
    if [[ -n $host_pid ]]; then
        kill -KILL -- "-$host_pid" "$host_pid" 2>/dev/null || true
        wait "$host_pid" 2>/dev/null || true
    fi
    if [[ -n $lab_pid ]]; then
        kill "$lab_pid" 2>/dev/null || true
        wait "$lab_pid" 2>/dev/null || true
    fi
    rm -f -- "$control/stop"
    rmdir -- "$control"
    if [[ -n $shared ]]; then
        rm -f -- "$shared"/{job.json,overlay.yaml,map_world.npz,map_world.tmp.npz}
        rmdir -- "$shared"
    fi
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if [[ $profile == map_dump ]]; then
    shared=$(mktemp -d "$root/experiments/isaac/.map-dump.XXXXXX")
    dump_dir="/ws/${shared#"$root"/}"
    PYTHONPATH="$root/experiments/isaac${PYTHONPATH:+:$PYTHONPATH}" python3 - \
        "$map" "$overlay" "$out/job.json" <<'PY'
import json
import sys
from pathlib import Path

from drive_core import load_map

source, overlay, path = sys.argv[1:]
track = load_map(source)
pose = track.start
job = dict(map=str(track.source), profile=Path(overlay).stem if overlay else "map_dump",
           odom_origin=[pose.x, pose.y, pose.yaw], spawn_z=pose.z)
Path(path).write_text(json.dumps(job, indent=2) + "\n")
PY
    cp -- "$out/job.json" "$shared/job.json"
    if [[ -n $overlay ]]; then cp -- "$overlay" "$shared/overlay.yaml"; fi
fi
mkfifo "$control/stop"
exec {keepalive}<>"$control/stop"

ISAAC_DRIVE_PROFILE=$profile ROS_DOMAIN_ID=$domain setsid "$isaac/python.sh" \
    "$root/experiments/isaac/isaac_drive.py" "${drive_args[@]}" --map "$map" --out "$out" \
    --bind "$bind" --port "$port" {keepalive}>&- >"$out/isaac.log" 2>&1 &
host_pid=$!

# Reuse the bench lock inside the container and keep DDS on its loopback interface.
container_script='
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd /ws
exec 9>/tmp/line-follower-bench-"$ROS_DOMAIN_ID".lock
flock -n 9 || { echo "Bench domain $ROS_DOMAIN_ID is in use" >&2; exit 1; }
export LF_PARAMS=/ws/src/line_follower/config/rule.yaml LF_X_FAR=0.85
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
run_dir=$(mktemp -d /tmp/isaac-drive.XXXXXX)
export ROS_LOG_DIR="$run_dir/ros" GZ_PARTITION="isaac_drive_${run_dir##*.}"
children=()
stop() {
    trap - EXIT INT TERM HUP
    if (("${#children[@]}")); then
        # Each process group includes any xfar truth logger subprocesses.
        for pid in "${children[@]}"; do kill -TERM -- "-$pid" "$pid" 2>/dev/null || true; done
        sleep 0.3
        for pid in "${children[@]}"; do kill -KILL -- "-$pid" "$pid" 2>/dev/null || true; done
        wait "${children[@]}" 2>/dev/null || true
    fi
    rm -rf -- "$run_dir"
}
trap stop EXIT
trap "exit 1" INT TERM HUP
relay_args=()
if [[ $1 == map_dump ]]; then
    relay_args+=(--map-dump)
    setsid python3 experiments/isaac/run_map_dump.py $FOLLOWER_ARGS &
elif [[ $1 == run_user ]]; then
    setsid python3 experiments/user_base/run_user.py $FOLLOWER_ARGS &
elif [[ $1 == run_split ]]; then
    # the split nodes talk over line_follower_interfaces messages (built in /ws/install)
    [[ -f install/setup.bash ]] && source install/setup.bash
    setsid python3 experiments/user_base/run_split.py $FOLLOWER_ARGS &
else
    setsid python3 experiments/user_base/run_xfar.py --xfar 0.85 $FOLLOWER_ARGS &
fi
follower=$!
children+=("$follower")
setsid python3 experiments/isaac/isaac_relay.py --host "$2" --port "$3" --startup-seconds "${ISAAC_STARTUP_SECONDS:-900}" "${relay_args[@]}" &
relay=$!
children+=("$relay")
# The watcher needs an explicit stdin redirect because bash backgrounds on /dev/null.
read -r _ <&3 &
watcher=$!
children+=("$watcher")
status=0
ended=
wait -n -p ended "$follower" "$relay" "$watcher" || status=$?
if [[ ${ended:-} == "$follower" ]]; then
    echo "Follower exited before the relay" >&2
    exit 1
fi
if [[ ${ended:-} == "$watcher" ]]; then exit 0; fi
if [[ $1 == map_dump && $status == 0 ]]; then
    kill -TERM "$follower"
    for ((i=0; i<200; i++)); do
        kill -0 "$follower" 2>/dev/null || break
        sleep 0.1
    done
    if kill -0 "$follower" 2>/dev/null; then
        echo "Follower timed out during final map dump" >&2
        exit 1
    fi
    wait "$follower"
fi
exit "$status"
'
# Preserve original stdin on fd 3 for the EOF watcher.
# Perception log for the planning team (per-frame noisy observations), on by default when the
# run directory is inside the repo (mounted at /ws). ISAAC_PERCEPTION_LOG=0 turns it off.
plog=
if [[ ${ISAAC_PERCEPTION_LOG:-1} != 0 && $out == "$root"/* ]]; then plog=/ws/${out#"$root"/}/perception_log; fi
docker_args=(docker exec -i -e "LAB_NAME=$lab" -e "ROS_DOMAIN_ID=$domain" -e "FOLLOWER_ARGS=${ISAAC_FOLLOWER_ARGS:-}" -e "ISAAC_HOLD=${ISAAC_HOLD:-1}" -e "LF_PERCEPTION_LOG=$plog"
    -e "ISAAC_STARTUP_SECONDS=${ISAAC_STARTUP_SECONDS:-900}"
    -e "ISAAC_DUMP_DIR=$dump_dir"
    -e ROS2CLI_NO_DAEMON=1 "$lab" bash -c "exec 3<&0
$container_script" isaac-drive "$profile" "$relay_host" "$port")
quoted=
for arg in "${docker_args[@]}"; do
    quoted+=" '${arg//\'/\'\\\'\'}'"
done
sg docker -c "$quoted" 0<"$control/stop" {keepalive}>&- >"$out/lab.log" 2>&1 &
lab_pid=$!

while [[ ! -f $out/result.json ]]; do
    if ! kill -0 "$host_pid" 2>/dev/null; then
        [[ -f $out/result.json ]] && break
        echo "Isaac exited without a result. See $out/isaac.log" >&2
        exit 1
    fi
    if ! kill -0 "$lab_pid" 2>/dev/null; then
        status=0
        wait "$lab_pid" || status=$?
        lab_pid=
        if (( status != 0 )); then
            echo "Lab process failed. See $out/lab.log" >&2
            exit 1
        fi
        # On success the relay can see EOF before the renderer writes its result.
        for ((i=0; i<100; i++)); do
            [[ ! -f $out/result.json ]] || break
            sleep 0.1
        done
        [[ -f $out/result.json ]] || { echo "Relay exited without a result. See $out/isaac.log" >&2; exit 1; }
        break
    fi
    sleep 0.2
done
python3 - "$out/result.json" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
result = json.loads(path.read_text())
print(path)
print(result["reason"])
raise SystemExit(int(result["reason"].startswith("error:")))
PY
if [[ $profile == map_dump ]]; then
    if [[ -n $lab_pid ]] && ! wait "$lab_pid"; then
        echo "Lab process failed during map export. See $out/lab.log" >&2
        exit 1
    fi
    lab_pid=
    cp -- "$shared/map_world.npz" "$out/map_world.tmp.npz"
    mv -- "$out/map_world.tmp.npz" "$out/map_world.npz"
fi
