#!/bin/bash
# Headless Gazebo smoke test of the competition launch (system-level T-01 / T-03 / T-04).
# Usage: tools/sim_smoke_test.sh [run_seconds]      (default 40; the 20 s start gate needs > 25)
# Isolated with ROS_DOMAIN_ID / GZ_PARTITION; stops only its own launch process group on exit.
set -u
RUN_SEC=${1:-40}
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
export GZ_PARTITION=lf_smoke_$$
source /opt/ros/jazzy/setup.bash
source "${INSTALL_DIR:-$WS/install}/setup.bash"
LOG="$(mktemp -d)/launch.log"

set -m  # the background job gets its own process group (pgid == $!)
ros2 launch line_follower_bringup competition.launch.py headless:=true > "$LOG" 2>&1 &
LPID=$!
set +m

cleanup() {
  kill -INT -"$LPID" 2>/dev/null
  for _ in $(seq 1 20); do pgrep -g "$LPID" > /dev/null || break; sleep 0.5; done
  kill -KILL -"$LPID" 2>/dev/null
  echo "--- leftover in launch group: $(pgrep -g "$LPID" | wc -l)"
  echo "--- mission transitions"; grep -E "mission state" "$LOG"
  echo "--- errors"; grep -iE "error|exception|fatal|died" "$LOG" || echo none
  echo "--- full log: $LOG"
}
trap cleanup EXIT

T0=$(date +%s)
# -k: some ros2 CLI verbs (e.g. topic hz) ignore SIGTERM, so follow up with SIGKILL.
cli() { timeout -k 2 5 "$@" 2>&1; }
sample() {
  echo "--- t=$(( $(date +%s) - T0 ))s"
  echo "mission state : $(cli ros2 topic echo --once /mission/state --field state | head -1)"
  echo "cmd_vel       : $(cli ros2 topic echo --once /cmd_vel --field linear.x | head -1)" \
       "/ $(cli ros2 topic echo --once /cmd_vel --field angular.z | head -1)"
  echo "odom x        : $(cli ros2 topic echo --once /odom --field pose.pose.position.x | head -1)"
  echo "safety reason : $(cli ros2 topic echo --once /safety/status --field stop_reason | head -1)"
}
publishers() {
  echo "$1 publishers: $(cli ros2 topic info -v "$1" | awk '
    /^Publisher count/ {p = 1; next} /^Subscription count/ {p = 0}
    p && /Node name/ {printf "%s ", $3}')"
}
wait_until() { while (( $(date +%s) - T0 < $1 )); do sleep 1; done; }

wait_until 8
sample
publishers /cmd_vel
publishers /camera/camera_info
echo "camera rate: $(cli ros2 topic hz /camera/image_raw | grep average | tail -1)"
wait_until "$RUN_SEC"
sample
