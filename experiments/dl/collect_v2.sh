#!/usr/bin/env bash
set -euo pipefail
export LAB_NAME=lf_b ROS_DOMAIN_ID=12 GZ_PARTITION=lf_b
root=experiments/dl/runs/dataset-v2
mkdir -p "$root"/{worlds,train,val,test}
for split in train val test; do
  for path in experiments/dl/runs/dataset-v1/"$split"/*; do
    [[ -f "$path/summary.json" ]] || continue
    ln -sfnT "../../dataset-v1/$split/${path##*/}" "$root/$split/${path##*/}"
  done
done

collect() {
  local split=$1 difficulty=$2 seed=$3 count=$4
  local name="${difficulty}_${seed}" world dest
  world="$root/worlds/$name.sdf"
  dest="$root/$split/$name"
  [[ ! -f "$dest/summary.json" ]] || return 0
  python3 experiments/worlds/gen_world.py --seed "$seed" --difficulty "$difficulty" --obstacles 0 --output "$world"
  for attempt in 1 2 3; do
    if [[ -e "$dest" ]]; then mv "$dest" "$dest.incomplete.$(date +%s%N)"; fi
    if sg docker -c "docker exec -e LAB_NAME -e ROS_DOMAIN_ID -e GZ_PARTITION lf_b bash -c 'source /opt/ros/jazzy/setup.bash && python3 -m experiments.dl.collect --world $world --out $dest --count $count --seed $((1000+seed)) --settle 0.18'"; then
      return 0
    fi
    echo "$name: error $attempt/3" >&2
  done
  return 1
}
collect train easy 401 240
collect train normal 402 240
collect train hard 403 240
collect val normal 451 120
