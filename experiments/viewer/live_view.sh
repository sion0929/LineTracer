#!/bin/bash
# Official track + rule follower + 640x480 camera window. Run inside line_follower_lab. Ctrl+C or close to stop.
source /opt/ros/jazzy/setup.bash
cd /ws
export ROS_DOMAIN_ID=99 GZ_PARTITION=live_view_$$ LAB_NAME=line_follower_lab
export GZ_SIM_RESOURCE_PATH=/ws/src/line_follower_bringup/models
export DISPLAY=${DISPLAY:-:0}
trap 'kill 0' EXIT
gz sim -s -r --headless-rendering /ws/src/line_follower_bringup/worlds/line_track.sdf > /tmp/live_gz.log 2>&1 &
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/ws/src/line_follower_bringup/config/ros_gz_bridge.yaml > /tmp/live_bridge.log 2>&1 &
ros2 run ros_gz_image image_bridge /camera/image_raw > /tmp/live_img.log 2>&1 &
python3 /ws/experiments/viewer/live_view.py &
sleep ${START_DELAY:-12}
LF_PARAMS=/ws/src/line_follower/config/rule.yaml python3 /ws/experiments/user_base/run_user.py > /tmp/live_follower.log 2>&1 &
wait
