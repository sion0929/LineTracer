#!/bin/bash
# usage: experiments/run.sh NAME [follower args...]   (runs one headless trial in the lab container)
set -euo pipefail
name=${1:?usage: experiments/run.sh NAME [follower args...]}
shift
read -r -a eval_args <<< "${EVAL_ARGS:-}"

command=(docker exec -e LAB_NAME -e ROS_DOMAIN_ID -e GZ_PARTITION
    "${LAB_NAME:-line_follower_lab}" bash -c
    'source /opt/ros/jazzy/setup.bash && source /ws/install_lab/setup.bash &&
     cd /ws && exec python3 experiments/run_eval.py "$@"' run-eval
    --name "$name" --plot "${eval_args[@]}" -- "$@")
if docker info >/dev/null 2>&1; then
    "${command[@]}"
else
    # sg runs its command through a shell. Quote each argument for that shell.
    quoted=
    for arg in "${command[@]}"; do
        quoted+=" '${arg//\'/\'\\\'\'}'"
    done
    sg docker -c "$quoted"
fi
