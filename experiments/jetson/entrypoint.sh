#!/usr/bin/env bash
# Container entry point: ROS 2 Jazzy + the line_follower workspace overlay.
set -e

source /opt/ros/jazzy/setup.bash
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

# Fast DDS is already the L4T Jazzy default; keep an explicit override possible.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

exec "$@"
