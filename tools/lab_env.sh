#!/bin/bash
# Evaluation-like Gazebo environment (official PDF p.3): course image rp/ros2:jazzy-lab + provided world, with GUI.
# The workspace is mounted at /ws (no spaces inside the container) and built into build_lab/ install_lab/,
# so container and host builds never clash. Default Docker network: no crosstalk with other ROS/Gazebo stacks.
#
#   tools/lab_env.sh up             start the container (idempotent)
#   tools/lab_env.sh build          colcon build inside the container
#   tools/lab_env.sh test           colcon test + results inside the container
#   tools/lab_env.sh smoke [sec]    headless smoke test (tools/sim_smoke_test.sh) inside the container
#   tools/lab_env.sh check-sensors|check-motion|check-start-stop   headless acceptance checks
#   tools/lab_env.sh checkpoint-a   tests + sensors/motion/controller-loss/guard-exit checks
#   tools/lab_env.sh world          PDF p.3 verbatim: gz sim -r worlds/line_track.sdf (world only)
#   tools/lab_env.sh competition    ros2 launch line_follower_bringup competition.launch.py [args]
#   tools/lab_env.sh shell          interactive shell (ROS + workspace sourced)
#   tools/lab_env.sh gcs            visible GCS + headless Gazebo (gcs-stop or Ctrl-C to stop)
#   tools/lab_env.sh check-gcs      automated D-11 / D-12 communication and offscreen UI checks
#   tools/lab_env.sh check-bag      D-13 recording, event extraction, UI scrub and safe replay
#   tools/lab_env.sh stop|gcs-stop  stop only the managed GCS session
#   tools/lab_env.sh down           remove the container
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME=line_follower_lab
IMAGE=${LAB_IMAGE:-rp/ros2:jazzy-lab}
SETUP='source /opt/ros/jazzy/setup.bash; [ -f /ws/install_lab/setup.bash ] && source /ws/install_lab/setup.bash'

running() { [ "$(docker inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null)" = true ]; }

up() {
  running && return
  docker rm -f "$NAME" > /dev/null 2>&1 || true
  local gpu=()
  if command -v nvidia-smi > /dev/null; then gpu+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all); fi
  if [ -e /dev/dri ]; then gpu+=(--device /dev/dri); fi
  docker run -d --init --name "$NAME" \
    --user "$(id -u):$(id -g)" -e HOME=/tmp/home \
    -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    "${gpu[@]}" --shm-size=1g \
    -v "$WS":/ws -w /ws \
    --entrypoint bash "$IMAGE" -c 'mkdir -p "$HOME" && exec sleep infinity' > /dev/null
  echo "started $NAME ($IMAGE)"
}

TTY=-i
if [ -t 0 ]; then TTY=-it; fi
in_lab() { up; docker exec $TTY "$NAME" bash -c "$SETUP; $1"; }

case "${1:-}" in
  up) up ;;
  build) in_lab "colcon --log-base log_lab build --build-base build_lab --install-base install_lab --cmake-args -DCMAKE_BUILD_TYPE=Release" ;;
  test) in_lab "export ROS_DOMAIN_ID=78; colcon --log-base log_lab test --build-base build_lab --install-base install_lab && colcon test-result --test-result-base build_lab --verbose" ;;
  gcs) in_lab "python3 tools/gcs_session.py ${*:2}" ;;
  gcs-stop) in_lab "python3 tools/gcs_session.py stop" ;;
  measure-components) in_lab "python3 tools/measure_components.py" ;;
  check-teardown) in_lab "python3 tools/check_teardown.py ${*:2}" ;;
  check-drive) in_lab "python3 tools/check_drive.py ${*:2}" ;;
  check-d07) in_lab "python3 tools/check_d07.py ${*:2}" ;;
  check-d06) in_lab "python3 tools/check_d06.py ${*:2}" ;;
  check-d05) in_lab "python3 tools/check_d05.py" ;;
  check-camera-rate) in_lab "python3 tools/check_camera_rate.py" ;;
  check-bag) in_lab "python3 tools/check_bag.py" ;;
  check-gcs) in_lab "python3 tools/check_gcs.py" ;;
  check-sensors) in_lab "python3 tools/checkpoint_a.py sensors" ;;
  check-motion) in_lab "python3 tools/checkpoint_a.py motion" ;;
  check-guard-exit) in_lab "python3 tools/checkpoint_a.py guard-exit" ;;
  check-start-stop) in_lab "python3 tools/checkpoint_a.py start-stop" ;;
  checkpoint-a)
    failed=0
    for check in test check-sensors check-motion check-start-stop check-guard-exit; do
      if "$0" "$check"; then echo "CHECKPOINT A $check: PASS";
      else echo "CHECKPOINT A $check: FAIL"; failed=1; fi
    done
    if [ "$failed" = 0 ]; then echo "CHECKPOINT A: PASS (D-01 D-02 D-03 D-04)";
    else echo "CHECKPOINT A: FAIL"; fi
    exit "$failed" ;;
  smoke) in_lab "INSTALL_DIR=/ws/install_lab ROS_DOMAIN_ID=77 tools/sim_smoke_test.sh ${2:-40}" ;;
  world) in_lab "cd /ws/src/line_follower_bringup && export GZ_SIM_RESOURCE_PATH=\$PWD/models && gz sim -r worlds/line_track.sdf" ;;
  competition) in_lab "ros2 launch line_follower_bringup competition.launch.py ${*:2}" ;;
  shell) in_lab "exec bash" ;;
  stop) in_lab "python3 tools/gcs_session.py stop" ;;
  down) docker rm -f "$NAME" ;;
  *) sed -n '2,15p' "$0"; exit 1 ;;
esac
