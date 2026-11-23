#!/usr/bin/env bash
# Run from the b-dlseg root after building the lf_b container.
set -euo pipefail
export LAB_NAME=lf_b ROS_DOMAIN_ID=12 GZ_PARTITION=lf_b
generator=${1:?Pass the path to Codex-3 gen_world.py}
root=experiments/dl/runs/dataset-v1
mkdir -p "$root/worlds"

collect() {
  local split=$1 name=$2 count=$3 seed=$4
  shift 4
  local dest="$root/$split/$name"
  if [[ -f "$dest/summary.json" ]]; then
    return
  fi
  if [[ -e "$dest" ]]; then
    mv "$dest" "$dest.incomplete.$(date +%s%N)"
  fi
  sg docker -c "docker exec -e LAB_NAME -e ROS_DOMAIN_ID -e GZ_PARTITION lf_b bash -c 'source /opt/ros/jazzy/setup.bash && python3 -m experiments.dl.collect --out $dest --count $count --seed $seed --settle 0.18 $*'"
}

random_map() {
  local split=$1 difficulty=$2 seed=$3 obstacle_count=$4 count=$5
  local name="${difficulty}_${seed}" world
  world="$root/worlds/$name.sdf"
  python3 "$generator" --seed "$seed" --difficulty "$difficulty" --obstacles "$obstacle_count" --output "$world"
  collect "$split" "$name" "$count" "$((1000 + seed))" --world "$world"
}

random_map train easy 101 3 420
random_map train normal 102 4 420
random_map train hard 103 5 420
random_map train normal 104 4 420
random_map train hard 105 5 420
random_map val normal 201 4 180
random_map test easy 301 3 120
random_map test normal 302 4 120
random_map test hard 303 5 120
collect test official 180 1200
