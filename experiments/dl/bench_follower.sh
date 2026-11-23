#!/usr/bin/env bash
# Run from the b-dlseg root. Each benchmark row gets its own ROS/Gazebo context.
set -euo pipefail
export LAB_NAME=lf_b ROS_DOMAIN_ID=12 GZ_PARTITION=lf_b
candidate=${1:?Provide a unique candidate name}
shift
if [[ ! "$candidate" =~ ^[a-zA-Z0-9_-]+$ ]]; then
  echo 'Candidate name must use letters, digits, underscores or hyphens' >&2
  exit 2
fi
output="experiments/dl/runs/follower-bench/$candidate"
official=src/line_follower_bringup/worlds/line_track.sdf
maps=("$official" "$official" "$official")
seeds=(20 26 33)
while IFS= read -r map; do
  case "$map" in ''|\#*) continue ;; esac
  file=${map##*/}
  IFS=_ read -r difficulty seed obstacles <<< "${file%.sdf}"
  if [[ " ${seeds[*]} " != *" $seed "* ]]; then continue; fi
  python3 experiments/worlds/gen_world.py --seed "$seed" --difficulty "$difficulty" \
    --obstacles "${obstacles#obs}" --output "$map"
  maps+=("$map")
done < experiments/worlds/eval_set_G2.txt
[[ "${#maps[@]}" == 6 ]]
flags=(--reactive-recovery --ld-min 0.05 --ld-gain 0.2 --alat 0.10 --vmax 0.10
       --wmax 0.8 --waccel 1.6 --accel 0.30 --decel-cmd 0.40 "$@")
# Values are passed unexpanded by the inner shell; the Python CLI parses the profile.
printf -v invocation '%q ' python3 experiments/bench.py "${maps[@]}" --backend obstacle --off 0.12 \
  --follower experiments/dl/follower_dl.py --profile "$candidate=${flags[*]}" \
  --output "$output" --jobs "${DL_BENCH_JOBS:-2}" --limit 300 --stall 20
printf -v command '%q' "source /opt/ros/jazzy/setup.bash && source /ws/install_lab/setup.bash && $invocation"
sg docker -c "docker exec -e LAB_NAME -e ROS_DOMAIN_ID -e GZ_PARTITION \
  -e PYTHONPATH=/ws/experiments/dl/runs/lab-runtime:/ws lf_b bash -c $command"
